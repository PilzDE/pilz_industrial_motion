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

#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_interface/planning_interface.h>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>

#include "pilz_extensions/joint_limits_extension.h"
#include "pilz_trajectory_generation/limits_container.h"
#include "pilz_trajectory_generation/trajectory_functions.h"

namespace pilz {

/**
 * @brief Base class of trajectory generators
 *
 * Note: All derived classes cannot have a start velocity
 */
class TrajectoryGenerator
{
public:

  TrajectoryGenerator(const robot_model::RobotModelConstPtr& robot_model,
                      const pilz::LimitsContainer& planner_limits)
    :robot_model_(robot_model),
      planner_limits_(planner_limits)
  {
  }

  virtual ~TrajectoryGenerator(){}

  /**
   * @brief generate robot trajectory with given sampling time
   * @param req: motion plan request
   * @param res: motion plan response
   * @param sampling_time: sampling time of the generate trajectory (default 8ms)
   * @return motion plan succeed/fail, detailed information in motion plan responce
   */
  virtual bool generate(const planning_interface::MotionPlanRequest& req,
                        planning_interface::MotionPlanResponse&  res,
                        double sampling_time=0.008) = 0;

protected:
  /**
   * @brief This class is used to extract needed information from motion plan request.
   */
  class MotionPlanInfo
  {
  public:
    std::string group_name;
    std::string link_name;
    Eigen::Affine3d start_pose;
    Eigen::Affine3d goal_pose;
    std::map<std::string, double> start_joint_position;
    std::map<std::string, double> goal_joint_position;
    std::pair<std::string, Eigen::Vector3d> circ_path_point;
  };

  /**
   * @brief Validate the motion plan request based on the common requirements of trajectroy generator
   * Checks that:
   *    - req.max_velocity_scaling_factor [0.0001, 1], moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN on failure
   *    - req.max_acceleration_scaling_factor [0.0001, 1] , moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN on failure
   *    - req.group_name is a JointModelGroup of the Robotmodel, moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME on failure
   *    - req.start_state.joint_state is not empty, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE on failure
   *    - req.start_state.joint_state is within the limits, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE on failure
   *    - req.start_state.joint_state is all zero, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE on failure
   *    - req.goal_constraints must have exactly 1 defined cartesian oder joint constraint
   *      moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on failure
   * A joint goal is checked for:
   *    - StartState joint-names matching goal joint-names, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on failure
   *    - Beeing defined in the req.group_name JointModelGroup
   *    - Beeing with the defined limits
   * A cartesian goal is checked for:
   *    - A defined link_name for the constraint, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on failure
   *    - Matching link_name for position and orientation constraints, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on failure
   *    - A IK solver exists for the given req.group_name and constraint link_name, moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION on failure
   *    - A goal pose define in position_constraints[0].constraint_region.primitive_poses, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on failure
   * @param req: motion plan request
   * @param err: moveit error code
   * @return: true if succeed
   */
  virtual bool validateRequest(const planning_interface::MotionPlanRequest& req,
                               moveit_msgs::MoveItErrorCodes& error_code) const;

  /**
   * @brief Validate that the start state of the request matches the requirements of the trajectory generator
   *
   * These requirements are:
   *     - Names of the joints and given joint position match in size and are non-zero
   *     - The start state is withing the position limits
   *     - The start state velocity is below TrajectoryGenerator::VELOCITY_TOLERANCE

   * @return true if the start state fullfills all requirements
   * @return false if the start state violates one requirement
   */
  virtual bool validateStartState(const planning_interface::MotionPlanRequest &req,
                                  moveit_msgs::MoveItErrorCodes &error_code) const;

  /**
   * @brief build cartesian velocity profile for the path
   *
   * Uses the path to get the cartesian length and the angular distance from start to goal.
   * The trap profile returns uses the longer distance of translational and rotational motion.
   */
  virtual std::unique_ptr<KDL::VelocityProfile> cartesianTrapVelocityProfile(
      const planning_interface::MotionPlanRequest &req,
      const MotionPlanInfo &plan_info,
      const std::unique_ptr<KDL::Path> &path) const;

  /**
   * @brief Extract needed information from a motion plan request in order to simplify
   * further usages.
   * @param req: motion plan request
   * @param info: information extracted from motion plan request which is necessary for the planning
   * @param error_code: MoveItErrorCodes which indicates the detailed error
   * @return: true if planning information is successfully extracted
   */
  virtual bool extractMotionPlanInfo(const planning_interface::MotionPlanRequest& req,
                                     MotionPlanInfo& info,
                                     moveit_msgs::MoveItErrorCodes& error_code) const = 0;

  /**
   * @brief set MotionPlanResponse from joint trajectory
   * @param req: MotionPlanRequest
   * @param res: MotionPlanResponse
   * @param joint_trajectory
   * @param err_code
   */
  bool setResponse(const planning_interface::MotionPlanRequest& req,
                   planning_interface::MotionPlanResponse& res,
                   const trajectory_msgs::JointTrajectory& joint_trajectory,
                   const moveit_msgs::MoveItErrorCodes& err_code,
                   const ros::Time &planning_start) const;


protected:
  const robot_model::RobotModelConstPtr robot_model_;
  const pilz::LimitsContainer planner_limits_;
  static constexpr double MIN_SCALING_FACTOR {0.0001};
  static constexpr double VELOCITY_TOLERANCE {1e-8};
};

/**
 * @class TrajectoryGeneratorException
 * @brief A base class for all trajectory generator exceptions inheriting from std::runtime_exception
 */
class TrajectoryGeneratorException : public std::runtime_error
{
  public:
    TrajectoryGeneratorException(const std::string error_desc) : std::runtime_error(error_desc) {}
};

/**
 * @class TrajectoryGeneratorInvalidLimitsException
 * @brief Thown when one of the required joint limit is not set
 */
class TrajectoryGeneratorInvalidLimitsException : public TrajectoryGeneratorException
{
  public:
    TrajectoryGeneratorInvalidLimitsException(const std::string error_desc) : TrajectoryGeneratorException(error_desc) {}
};

}
#endif // TRAJECTORYGENERATOR_H
