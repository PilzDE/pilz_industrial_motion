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
#ifndef COMMAND_LIST_MANAGER_H
#define COMMAND_LIST_MANAGER_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include "pilz_msgs/MotionSequenceRequest.h"
#include "pilz_trajectory_generation/trajectory_blender.h"

namespace pilz_trajectory_generation {

/**
 * @brief The CommandListManager class
 * This class can create a smooth trajectory from a given list of motion commands.
 * The trajectory generated from the motion commands are blended with each other within a blend radius given
 * within the MotionSequenceRequest.
 */
class CommandListManager {

public:

  /**
   * @brief CommandListManager
   * @param model The robot model
   */
  CommandListManager(const ros::NodeHandle& nh, const robot_model::RobotModelConstPtr& model);

  /**
   * @brief Returns a full trajectory consistenting of planned trajectory blended with each other in the given blend_radius
   * @param planning_scene The current planning scene
   * @param req_list List of motion requests. Contains PTP, LIN and CIRC commands.
   * @param[out] res The resulting trajectory
   * @return True if the generation was successful, false otherwise
   */
  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const pilz_msgs::MotionSequenceRequest& req_list,
             planning_interface::MotionPlanResponse &res);

private:
  /**
   * @brief Validate if the request list fullfills the following conditions
   * - All request are about the same group
   * - All blending radii are non negative
   * - The blending radius of the last request is 0
   * - Only the first request has a start state
   * @param req_list The request
   * @param res The response used to set the error code on validation error
   * @return True if all conditions hold, false otherwise
   */
  bool validateRequestList(const pilz_msgs::MotionSequenceRequest &req_list, planning_interface::MotionPlanResponse& res);

  /**
   * @brief Validates that two consecutive blending radii do not overlap
   * @param motion_plan_responses List of responses from the trajectory generator. Contains the trajectories.
   * @param radii List with the blending radii
   * @param group_name The group to consider
   * @return True if there is no overlap, false otherwise
   */
  bool validateBlendingRadiiDoNotOverlap(
      const std::vector<planning_interface::MotionPlanResponse>& motion_plan_responses,
      const std::vector<double>& radii,
      const std::string& group_name);

  /**
   * @brief solveRequests
   * @param planning_scene The planning_scene
   * @param req_list The motion plan request list
   * @param res The response used to set the error code on validation error
   * @param motion_plan_responses Essentially constains the generated trajectories
   * @param radii List of blending radii
   * @return True if trajectories for all request could be generated
   */
  bool solveRequests(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const pilz_msgs::MotionSequenceRequest &req_list,
                     planning_interface::MotionPlanResponse &res,
                     std::vector<planning_interface::MotionPlanResponse>& motion_plan_responses,
                     std::vector<double>& radii);

  /**
   * @brief Blends all trajectories inside motion_plan_responses with the given radii
   * @param motion_plan_responses Essentially constains the generated trajectories
   * @param radii List of blending radii
   * @param result_trajectory
   * @param res The response used to set the error code on validation error
   * @return True if blending succeeded, false otherwise. On false the res will contain the error code.
   */
  bool blend(const std::vector<planning_interface::MotionPlanResponse> &motion_plan_responses,
             const std::vector<double> &radii,
             robot_trajectory::RobotTrajectoryPtr& result_trajectory,
             planning_interface::MotionPlanResponse &res);

  /**
   * @brief The the name of the to frame (link) of the given group
   * @return name as string
   */
  const std::string& getTipFrame(const std::string& group_name);

private:
  /// Node handle
  ros::NodeHandle nh_;

  /// Robot model
  moveit::core::RobotModelConstPtr model_;

  /// TrajectoryBlender
  std::unique_ptr<pilz::TrajectoryBlender> blender_;
};

}

#endif // COMMAND_LIST_MANAGER_H
