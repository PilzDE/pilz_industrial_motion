/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "pilz_trajectory_generation/trajectory_generator.h"
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/velocityprofile_trap.hpp>

#include "pilz_trajectory_generation/limits_container.h"

namespace pilz{

bool TrajectoryGenerator::validateRequest(const planning_interface::MotionPlanRequest &req,
                                          moveit_msgs::MoveItErrorCodes &error_code) const
{
  // check the scaling factor
  if(req.max_velocity_scaling_factor > 1 || req.max_velocity_scaling_factor <= MIN_SCALING_FACTOR)
  {
    ROS_ERROR_STREAM("Velocity scaling factor of value " << req.max_velocity_scaling_factor
                     << " is not allowed. It must be in the range of [0.0001, 1].");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  if(req.max_acceleration_scaling_factor > 1 || req.max_acceleration_scaling_factor <= MIN_SCALING_FACTOR)
  {
    ROS_ERROR_STREAM("Acceleration scaling factor of value " << req.max_acceleration_scaling_factor
                     << " is not allowed. It must be in the range of [0.0001, 1].");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // check planning group
  if(!robot_model_->hasJointModelGroup(req.group_name))
  {
    ROS_ERROR_STREAM("Unknown planning group: " << req.group_name);
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }

  if(!validateStartState(req, error_code))
  {
    return false;
  }

  // check goal constraint
  if(req.goal_constraints.size()==0)
  {
    ROS_ERROR("Empty Goal Constraints.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }
  if(req.goal_constraints.size()>1)
  {
    ROS_ERROR("Too many Goal Constraints. Only one is allowed.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // no valid joint or Cartesian goal is given
  if(req.goal_constraints.front().joint_constraints.size() == 0 &&
     (req.goal_constraints.front().position_constraints.size() == 0 ||
      req.goal_constraints.front().orientation_constraints.size() == 0))
  {
    ROS_ERROR("Too few constraints for goal. Need one joint or Cartesian (position and orientation) constraint.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // too many goals
  if(req.goal_constraints.front().position_constraints.size()>1 ||
     req.goal_constraints.front().orientation_constraints.size()>1 ||
     (req.goal_constraints.front().joint_constraints.size() !=0 &&
      (req.goal_constraints.front().position_constraints.size() == 1 ||
       req.goal_constraints.front().orientation_constraints.size() == 1)) )
  {
    ROS_ERROR("Too many joint/position/orientation constrains for goal. Only one joint constraint or one Cartesian"
              " constraint (one position constraint and one orientation constraint) is allowed.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // check goal in joint space
  if(req.goal_constraints.front().joint_constraints.size() != 0)
  {
    //check the joint constraint
    for(auto const &joint_constraint : req.goal_constraints.front().joint_constraints)
    {
      // check corresponding state state
      if (std::find(req.start_state.joint_state.name.begin(),
                    req.start_state.joint_state.name.end(),
                    joint_constraint.joint_name) == req.start_state.joint_state.name.end())
      {
        ROS_ERROR_STREAM("Cannot find corresponding joint in start state for "
                         << joint_constraint.joint_name
                         << " in goal constraint.");
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
      }

      // check if the joint constraints are within this planning group
      if (!robot_model_->getJointModelGroup(req.group_name)->hasJointModel(joint_constraint.joint_name))
      {
        ROS_ERROR_STREAM(joint_constraint.joint_name << " does not belong to the planning group: " << req.group_name);
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
      }

      // verify the joint limit in goal constraint
      if(!planner_limits_.getJointLimitContainer().verifyPositionLimit(joint_constraint.joint_name, joint_constraint.position))
      {
        ROS_ERROR_STREAM("Joint \"" << joint_constraint.joint_name << "\" violates joint limits in goal constraints.");
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
      }
    }
  }
  //check the Cartesian space goal
  else if(req.goal_constraints[0].position_constraints.size() == 1 &&
          req.goal_constraints[0].orientation_constraints.size() == 1)
  {
    // check the link name
    if(req.goal_constraints.front().position_constraints.front().link_name.empty())
    {
      ROS_ERROR_STREAM("Link name of position constraint is empty. " );
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }
    if(req.goal_constraints.front().orientation_constraints.front().link_name.empty())
    {
      ROS_ERROR_STREAM("Link name of orientation constraint is empty. " );
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }
    if(req.goal_constraints.front().position_constraints.front().link_name !=
       req.goal_constraints.front().orientation_constraints.front().link_name)
    {
      ROS_ERROR_STREAM("Target link name of position constraint: "
                       << req.goal_constraints.front().position_constraints.front().link_name
                       << " and orientation constraint: "
                       <<  req.goal_constraints.front().orientation_constraints.front().link_name
                       << " are different." );
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }
    // check the link name can be solved by ik
    if(!robot_model_->getJointModelGroup(req.group_name)->
       canSetStateFromIK(req.goal_constraints.front().position_constraints.front().link_name))
    {
      ROS_ERROR_STREAM("No IK solver is available for link: "
                       << req.goal_constraints.front().position_constraints.front().link_name
                       <<". ");
      error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }
    // check if the needed pose is set
    if(req.goal_constraints.front().position_constraints.front().constraint_region.primitive_poses.size() == 0)
    {
      ROS_ERROR_STREAM("No primitive poses available in posiion constraints. ");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;

}

bool TrajectoryGenerator::validateStartState(const planning_interface::MotionPlanRequest &req,
                                             moveit_msgs::MoveItErrorCodes &error_code) const
{
  // check start state
  if(req.start_state.joint_state.name.size() == 0)
  {
    ROS_ERROR("Empty joint name of start state in planning request.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  if(req.start_state.joint_state.name.size() != req.start_state.joint_state.position.size())
  {
    ROS_ERROR("Joint state name and position do not match in start state.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  if(!planner_limits_.getJointLimitContainer().verifyPositionLimits(req.start_state.joint_state.name,
                                                                    req.start_state.joint_state.position))
  {
    ROS_ERROR("Joint state out of range in start state.") ;
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  // does not allow start velocity
  if(!std::all_of(req.start_state.joint_state.velocity.begin(), req.start_state.joint_state.velocity.end(),
                  [this](double v) { return std::fabs(v) < this->VELOCITY_TOLERANCE; }))
  {
    ROS_ERROR("Trajectory Generator does not allow non-zero start velocity.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}

bool TrajectoryGenerator::setResponse(const planning_interface::MotionPlanRequest &req,
                                      planning_interface::MotionPlanResponse &res,
                                      const trajectory_msgs::JointTrajectory &joint_trajectory,
                                      const moveit_msgs::MoveItErrorCodes &err_code,
                                      const ros::Time& planning_start) const
{
  // if invalid, return empty trajectory
  if(err_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    res.error_code_.val = err_code.val;
    if(res.trajectory_)
    {
      res.trajectory_->clear();
    }
    res.planning_time_ = (ros::Time::now() - planning_start).toSec();
    return false;
  }
  else
  {
    // convert trajectory_msgs::JointTrajectory to robot_trajectory::RobotTrajectory
    robot_trajectory::RobotTrajectoryPtr rt(new robot_trajectory::RobotTrajectory(robot_model_, req.group_name));
    moveit::core::RobotState start_rs(robot_model_);
    start_rs.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(req.start_state, start_rs, false);
    rt->setRobotTrajectoryMsg(start_rs,joint_trajectory);
    res.trajectory_ = rt;
    res.error_code_.val = err_code.val;
    res.planning_time_ = (ros::Time::now() - planning_start).toSec();
    return true;
  }
}

std::unique_ptr<KDL::VelocityProfile> TrajectoryGenerator::cartesianTrapVelocityProfile(
    const planning_interface::MotionPlanRequest &req,
    const MotionPlanInfo& plan_info,
    const std::unique_ptr<KDL::Path> &path) const
{
  std::unique_ptr<KDL::VelocityProfile> vp_trans(
        new KDL::VelocityProfile_Trap(
          req.max_velocity_scaling_factor*planner_limits_.getCartesianLimits().getMaxTranslationalVelocity(),
          req.max_acceleration_scaling_factor*planner_limits_.getCartesianLimits().getMaxTranslationalAcceleration()));

  if(path->PathLength() > std::numeric_limits<double>::epsilon()) // avoid division by zero
  {
    vp_trans->SetProfile(0, path->PathLength());
  }
  else
  {
    vp_trans->SetProfile(0, std::numeric_limits<double>::epsilon());
  }
  return std::move(vp_trans);
}

}
