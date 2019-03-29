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

#include "pilz_trajectory_generation/trajectory_generator_circ.h"
#include "pilz_trajectory_generation/path_circle_generator.h"

#include <cassert>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/utilities/error.h>
#include <kdl/utilities/utility.h>
#include <kdl/trajectory_segment.hpp>
#include <kdl/rotational_interpolation_sa.hpp>

namespace pilz {

TrajectoryGeneratorCIRC::TrajectoryGeneratorCIRC(const moveit::core::RobotModelConstPtr &robot_model,
                                                 const LimitsContainer &planner_limits)
  :TrajectoryGenerator::TrajectoryGenerator(robot_model, planner_limits)
{
  if(!planner_limits_.hasFullCartesianLimits())
  {
    throw TrajectoryGeneratorInvalidLimitsException("Cartesian limits are not fully set for CIRC trajectory generator.");
  }
}

TrajectoryGeneratorCIRC::~TrajectoryGeneratorCIRC()
{

}

bool TrajectoryGeneratorCIRC::generate(const planning_interface::MotionPlanRequest &req,
                                       planning_interface::MotionPlanResponse &res,
                                       double sampling_time)
{
  ROS_INFO("Start generation of CIRC trajectory!");

  ros::Time planning_begin = ros::Time::now();
  moveit_msgs::MoveItErrorCodes error_code;
  MotionPlanInfo plan_info;
  trajectory_msgs::JointTrajectory joint_trajectory;

  // validate the common requirements of motion plan request
  if(!validateRequest(req, error_code))
  {
    ROS_ERROR("Failed to validate the planning request of a CIRC command.");
    return setResponse(req, res, joint_trajectory, error_code, planning_begin);
  }
  // extract planning information from the motion plan request
  if(!extractMotionPlanInfo(req, plan_info, error_code))
  {
    ROS_ERROR("Cannot extract needed information from motion plan request.");
    return setResponse(req, res, joint_trajectory, error_code, planning_begin);
  }

  // create Cartesian path for circle
  std::unique_ptr<KDL::Path> path(setPathCIRC(plan_info, error_code));
  if(!path)
  {
    ROS_ERROR("Failed to set Cartesian path of the circle.");
    return setResponse(req, res, joint_trajectory, error_code, planning_begin);
  }

  // create velocity profile
  std::unique_ptr<KDL::VelocityProfile> vp(cartesianTrapVelocityProfile(req, plan_info, path));

  // combine path and velocity profile into Cartesian trajectory
  // with the third parameter set to false, KDL::Trajectory_Segment does not take
  // the ownship of Path and Velocity Profile
  KDL::Trajectory_Segment cart_trajectory(path.get(), vp.get(), false);

  // sample the Cartesian trajectory and compute joint trajectory using inverse kinematics
  if(!generateJointTrajectory(robot_model_,
                              planner_limits_.getJointLimitContainer(),
                              cart_trajectory,
                              plan_info.group_name,
                              plan_info.link_name,
                              plan_info.start_joint_position,
                              sampling_time,
                              joint_trajectory,
                              error_code))
  {
    ROS_ERROR("Failed to generate valid joint trajectory from the Cartesian path.");
  }
  else
  {
    ROS_INFO_STREAM("CIRC Trajectory with " << joint_trajectory.points.size() << " Points generated. Took "
                    << (ros::Time::now() - planning_begin).toSec() * 1000 << " ms.");
  }


  return setResponse(req, res, joint_trajectory, error_code, planning_begin);
}

bool TrajectoryGeneratorCIRC::validateRequest(const planning_interface::MotionPlanRequest &req,
                                              moveit_msgs::MoveItErrorCodes &error_code) const
{

  if(!TrajectoryGenerator::validateRequest(req, error_code))
  {
    return false;
  }

  // CIRC needs one position constraint as interim or center point
  if(! (req.path_constraints.name == "interim" ||
        req.path_constraints.name == "center") )
  {
    ROS_ERROR_STREAM("Undefined path constraint with the name: " << req.path_constraints.name << " for CIRC motion."
                     << "Only interim/center point is allowed.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }
  if(req.path_constraints.position_constraints.size() != 1)
  {
    ROS_ERROR("CIRC Trajectory Generator needs a valid path constraint.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }
  if(req.path_constraints.position_constraints.front().constraint_region.primitive_poses.size() != 1)
  {
    ROS_ERROR("CIRC Trajectory Generator needs a valid path constraint.");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  return true;
}

bool TrajectoryGeneratorCIRC::extractMotionPlanInfo(const planning_interface::MotionPlanRequest &req,
                                                    TrajectoryGenerator::MotionPlanInfo &info,
                                                    moveit_msgs::MoveItErrorCodes &error_code) const
{
  ROS_DEBUG("Extract necessary information from motion plan request.");

  info.group_name = req.group_name;
  std::string frame_id {robot_model_->getModelFrame()};

  // goal given in joint space
  if(req.goal_constraints.front().joint_constraints.size() != 0)
  {
    // TODO: link name from goal constraint and path constraint
    info.link_name = req.path_constraints.position_constraints.front().link_name;
    if(!robot_model_->hasLinkModel(info.link_name))
    {
      ROS_ERROR("Unknown link name of CIRC help point.");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
      return false;
    }


    if(req.goal_constraints.front().joint_constraints.size() !=
       robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames().size())
    {
      ROS_ERROR_STREAM("Number of joint constraint is not equal to the active joints in the planning group");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    for(const auto &joint_item : req.goal_constraints.front().joint_constraints)
    {
      info.goal_joint_position[joint_item.joint_name] = joint_item.position;
    }


    computeLinkFK(robot_model_,
                  info.link_name,
                  info.goal_joint_position,
                  info.goal_pose);

  }
  // goal given in Cartesian space
  else
  {
    info.link_name = req.goal_constraints.front().position_constraints.front().link_name;
    if(req.goal_constraints.front().position_constraints.front().header.frame_id.empty() ||
       req.goal_constraints.front().orientation_constraints.front().header.frame_id.empty())
    {
      ROS_WARN("Frame id is not set in position/orientation constraints of goal. Use model frame as default");
      frame_id = robot_model_->getModelFrame();
    }
    else
    {
      frame_id = req.goal_constraints.front().position_constraints.front().header.frame_id;
    }
    geometry_msgs::Pose goal_pose_msg;
    goal_pose_msg.position = req.goal_constraints.front().position_constraints.front()
        .constraint_region.primitive_poses.front().position;
    goal_pose_msg.orientation = req.goal_constraints.front().orientation_constraints.front().orientation;
    normalizeQuaternion(goal_pose_msg.orientation);
    tf::poseMsgToEigen(goal_pose_msg, info.goal_pose);
  }

  assert(req.start_state.joint_state.name.size() == req.start_state.joint_state.position.size());
  for(const auto& joint_name : robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames())
  {
    auto it {std::find(req.start_state.joint_state.name.cbegin(), req.start_state.joint_state.name.cend(), joint_name)};
    if (it == req.start_state.joint_state.name.cend())
    {
      ROS_ERROR_STREAM("Could not find joint \"" << joint_name << "\" of group \"" << req.group_name << "\" in start state of request");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
      return false;
    }
    size_t index = it - req.start_state.joint_state.name.cbegin();
    info.start_joint_position[joint_name] = req.start_state.joint_state.position[index];
  }

  computeLinkFK(robot_model_,
                info.link_name,
                info.start_joint_position,
                info.start_pose);

  //check goal pose ik before Cartesian motion plan starts
  std::map<std::string, double> ik_solution;
  if(!computePoseIK(robot_model_,
                    info.group_name,
                    info.link_name,
                    info.goal_pose,
                    frame_id,
                    info.start_joint_position,
                    ik_solution))
  {
    // LCOV_EXCL_START
    ROS_ERROR_STREAM("Failed to compute inverse kinematics for link: " << info.link_name << " of goal pose.");
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
    // LCOV_EXCL_STOP // not able to trigger here since lots of checks before are in place
  }


  Eigen::Vector3d circ_path_point;
  tf::pointMsgToEigen(req.path_constraints.position_constraints.front()
                      .constraint_region.primitive_poses.front().position, circ_path_point);

  info.circ_path_point.first = req.path_constraints.name;
  info.circ_path_point.second = circ_path_point;

  return true;
}

std::unique_ptr<KDL::Path> TrajectoryGeneratorCIRC::setPathCIRC(const MotionPlanInfo &info,
                                                                moveit_msgs::MoveItErrorCodes &error_code) const
{
  ROS_DEBUG("Set Cartesian path for CIRC command.");

  KDL::Frame start_pose, goal_pose;
  tf::transformEigenToKDL(info.start_pose, start_pose);
  tf::transformEigenToKDL(info.goal_pose, goal_pose);

  KDL::Vector path_point;
  tf::vectorEigenToKDL(info.circ_path_point.second, path_point);

  // pass the ratio of translational by rotational velocity as equivalent radius
  // to get a trajectory with rotational speed, if no (or very little) translational distance
  // The KDL::Path implementation chooses the motion with the longer duration (translation vs. rotation)
  // and uses eqradius as scaling factor between the distances.
  double eqradius = planner_limits_.getCartesianLimits().getMaxTranslationalVelocity()/
      planner_limits_.getCartesianLimits().getMaxRotationalVelocity();

  try
  {
    if(info.circ_path_point.first == "center")
    {
      return PathCircleGenerator::circleFromCenter(start_pose, goal_pose, path_point, eqradius);
    }
    else //if (info.circ_path_point.first == "interim")
    {
      return PathCircleGenerator::circleFromInterim(start_pose, goal_pose, path_point, eqradius);
    }
  }
  catch(KDL::Error_MotionPlanning_Circle_No_Plane &e)
  {
    ROS_ERROR_STREAM("Failed to create path object for circle. " << e.Description());
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return nullptr;
  }
  catch(KDL::Error_MotionPlanning_Circle_ToSmall &e)
  {
    ROS_ERROR_STREAM("Failed to create path object for circle. " << e.Description());
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return nullptr;
  }
  catch(Error_MotionPlanning_CenterPointDifferentRadius &e)
  {
    ROS_ERROR_STREAM("Failed to create path object for circle. " << e.Description());
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return nullptr;
  }
}

}

