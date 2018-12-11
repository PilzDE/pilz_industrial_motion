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

#include "pilz_trajectory_generation/trajectory_generator_ptp.h"
#include "ros/ros.h"
#include "eigen_conversions/eigen_msg.h"
#include "moveit/robot_state/conversions.h"

#include <iostream>

namespace pilz {

TrajectoryGeneratorPTP::TrajectoryGeneratorPTP(const robot_model::RobotModelConstPtr& robot_model,
                                               const LimitsContainer &planner_limits)
  :TrajectoryGenerator::TrajectoryGenerator(robot_model, planner_limits)
{

  if(!planner_limits_.hasJointLimits())
  {
    throw TrajectoryGeneratorInvalidLimitsException("joint limit not set");
  }

  joint_limits_ = planner_limits_.getJointLimitContainer();
  mostStrictLimit_ = joint_limits_.getCommonLimit();

  if(mostStrictLimit_.has_velocity_limits &&
     mostStrictLimit_.has_acceleration_limits &&
     mostStrictLimit_.has_deceleration_limits)
  {
    ROS_INFO("Initialized Point-to-Point Trajectory Generator.");
    ROS_DEBUG_STREAM("CommontLimit: " << "\n\t PositionLimits[" << mostStrictLimit_.min_position
                     << " ," << mostStrictLimit_.max_position << "]"
                     << "\n\t MaxVelocity " << mostStrictLimit_.max_velocity
                     << "\n\t MaxAcceleration" << mostStrictLimit_.max_acceleration
                     << "\n\t MaxDeceleration " << mostStrictLimit_.max_deceleration);
  }
  else
  {
    if(!mostStrictLimit_.has_velocity_limits)
    {
      ROS_ERROR("velocity limit not set");
      throw TrajectoryGeneratorInvalidLimitsException("velocity limit not set");
    }
    if(!mostStrictLimit_.has_acceleration_limits)
    {
      ROS_ERROR("acceleration limit not set");
      throw TrajectoryGeneratorInvalidLimitsException("acceleration limit not set");
    }
    if(!mostStrictLimit_.has_deceleration_limits)
    {
      ROS_ERROR("deceleration limit not set");
      throw TrajectoryGeneratorInvalidLimitsException("deceleartion limit not set");
    }
  }
}

TrajectoryGeneratorPTP::~TrajectoryGeneratorPTP()
{
}

bool TrajectoryGeneratorPTP::generate(const planning_interface::MotionPlanRequest &req,
                                      planning_interface::MotionPlanResponse& res,
                                      double sampling_time)
{
  ROS_INFO("Starting generation of PTP Trajectory!");

  // planning data
  ros::Time planning_begin = ros::Time::now();
  MotionPlanInfo plan_info;
  moveit_msgs::MoveItErrorCodes error_code;


  // validate the common requirements of motion plan request
  if(!validateRequest(req, error_code) )
  {
    trajectory_msgs::JointTrajectory joint_trajectory_empty;
    setResponse(req, res, joint_trajectory_empty, error_code, planning_begin);
    return false;
  }

  // extract planning information from the motion plan request
  if(!extractMotionPlanInfo(req, plan_info, error_code))
  {
    res.error_code_ = error_code;
    trajectory_msgs::JointTrajectory joint_trajectory_empty;
    setResponse(req, res, joint_trajectory_empty, error_code, planning_begin);
    return false;
  }

  // plan the ptp trajectory
  trajectory_msgs::JointTrajectory joint_trajectory;
  planPTP(plan_info.start_joint_position, plan_info.goal_joint_position, joint_trajectory,
          req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor, sampling_time);

  ROS_INFO_STREAM("PTP Trajectory with " << joint_trajectory.points.size() << " Points generated. Took "
                  << (ros::Time::now() - planning_begin).toSec() * 1000 << " ms.");

  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  setResponse(req, res, joint_trajectory, error_code, planning_begin);
  return true;
}


void TrajectoryGeneratorPTP::planPTP(const std::map<std::string, double>& start_pos,
                                     const std::map<std::string, double>& goal_pos,
                                     trajectory_msgs::JointTrajectory &joint_trajectory,
                                     const double &velocity_scaling_factor,
                                     const double &acceleration_scaling_factor,
                                     const double &sampling_time)
{
  // initialize joint names
  for(const auto& item : goal_pos)
  {
    joint_trajectory.joint_names.push_back(item.first);
  }

  // check if goal already reached
  bool goal_reached = true;
  for(auto const& goal: goal_pos)
  {
    if(fabs(start_pos.at(goal.first) - goal.second) >= MIN_MOVEMENT )
    {
      goal_reached = false;
      break;
    }
  }
  if(goal_reached)
  {
    ROS_INFO_STREAM("Goal already reached, set one goal point explicitly.");
    if(joint_trajectory.points.empty())
    {
      trajectory_msgs::JointTrajectoryPoint point;
      point.time_from_start =  ros::Duration(sampling_time);
      for(const std::string & joint_name : joint_trajectory.joint_names)
      {
        point.positions.push_back(start_pos.at(joint_name));
        point.velocities.push_back(0);
        point.accelerations.push_back(0);
      }
      joint_trajectory.points.push_back(point);
    }
    return;
  }

  // compute the fastest trajectory and choose the slowest joint as leading axis
  std::string leading_axis = joint_trajectory.joint_names.front();
  double max_duration = -1.0;

  std::map<std::string, VelocityProfile_ATrap> velocity_profile;
  for(const auto& joint_name : joint_trajectory.joint_names)
  {

    // create vecocity profile if necessary
    velocity_profile.insert(std::make_pair(
                              joint_name,
                              VelocityProfile_ATrap(
                                velocity_scaling_factor*joint_limits_.getLimit(joint_name).max_velocity,
                                acceleration_scaling_factor*joint_limits_.getLimit(joint_name).max_acceleration,
                                acceleration_scaling_factor*joint_limits_.getLimit(joint_name).max_deceleration)));

    velocity_profile.at(joint_name).SetProfile(start_pos.at(joint_name), goal_pos.at(joint_name));
    if(velocity_profile.at(joint_name).Duration() > max_duration)
    {
      max_duration = velocity_profile.at(joint_name).Duration();
      leading_axis = joint_name;
    }
  }

  // TODO throw exception?
  if(max_duration<=0)
  {
    ROS_ERROR("Trajectory duration is zero. It should not happen here.");
    joint_trajectory.points.clear();
    return;
  }

  // Full Synchronization
  // TODO!!! we assume all axes have same max_vel, max_acc, max_dec values
  // reset the velocity profile for other joints
  double acc_time = velocity_profile.at(leading_axis).FirstPhaseDuration();
  double const_time = velocity_profile.at(leading_axis).SecondPhaseDuration();
  double dec_time = velocity_profile.at(leading_axis).ThirdPhaseDuration();

  for(const auto& joint_name : joint_trajectory.joint_names)
  {
    if(joint_name != leading_axis)
    {
      // make full synchronization
      velocity_profile.at(joint_name).SetProfileAllDurations(start_pos.at(joint_name), goal_pos.at(joint_name),
                                                             acc_time,const_time,dec_time);
    }
  }

  // first generate the time samples
  std::vector<double> time_samples;
  for(double t_sample=0.0; t_sample<max_duration; t_sample+=sampling_time)
  {
    time_samples.push_back(t_sample);
  }
  // add last time
  time_samples.push_back(max_duration);

  // construct joint trajectory point
  for(double time_stamp : time_samples)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start =  ros::Duration(time_stamp);
    for(std::string & joint_name : joint_trajectory.joint_names)
    {
      point.positions.push_back(velocity_profile.at(joint_name).Pos(time_stamp));
      point.velocities.push_back(velocity_profile.at(joint_name).Vel(time_stamp));
      point.accelerations.push_back(velocity_profile.at(joint_name).Acc(time_stamp));
    }
    joint_trajectory.points.push_back(point);
  }
}


bool TrajectoryGeneratorPTP::extractMotionPlanInfo(const planning_interface::MotionPlanRequest& req,
                                                   MotionPlanInfo& info,
                                                   moveit_msgs::MoveItErrorCodes& error_code) const
{
  info.group_name = req.group_name;

  // extract start state information
  info.start_joint_position.clear();
  for(std::size_t i=0; i<req.start_state.joint_state.name.size(); ++i)
  {
    info.start_joint_position[req.start_state.joint_state.name[i]] = req.start_state.joint_state.position[i];
  }

  // extract goal
  info.goal_joint_position.clear();
  if(req.goal_constraints.at(0).joint_constraints.size() != 0)
  {
    for(std::size_t i=0; i<req.goal_constraints.at(0).joint_constraints.size(); ++i)
    {
      info.goal_joint_position[req.goal_constraints.at(0).joint_constraints[i].joint_name] =
          req.goal_constraints.at(0).joint_constraints[i].position;
    }
  }
  // slove the ik
  else
  {
    geometry_msgs::Point p = req.goal_constraints.at(0).position_constraints.at(0).
        constraint_region.primitive_poses.at(0).position;
    p.x -= req.goal_constraints.at(0).position_constraints.at(0).target_point_offset.x;
    p.y -= req.goal_constraints.at(0).position_constraints.at(0).target_point_offset.y;
    p.z -= req.goal_constraints.at(0).position_constraints.at(0).target_point_offset.z;

    geometry_msgs::Pose pose;
    pose.position = p;
    pose.orientation = req.goal_constraints.at(0).orientation_constraints.at(0).orientation;
    Eigen::Isometry3d pose_eigen;
    tf::poseMsgToEigen(pose,pose_eigen);
    if(!computePoseIK(robot_model_,
                      req.group_name,
                      req.goal_constraints.at(0).position_constraints.at(0).link_name,
                      pose_eigen,
                      robot_model_->getModelFrame(),
                      info.start_joint_position,
                      info.goal_joint_position))
    {
      ROS_ERROR("No IK solution for goal pose.");
      error_code.val =  moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }
  }

  return true;
}

}
