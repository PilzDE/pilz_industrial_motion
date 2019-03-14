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

#include <boost/optional.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include "pilz_msgs/MotionSequenceRequest.h"
#include "pilz_trajectory_generation/trajectory_blender.h"
#include "pilz_trajectory_generation/plan_components_builder.h"
#include "pilz_trajectory_generation/trajectory_generation_exceptions.h"

namespace pilz_trajectory_generation
{

using RobotTrajVec_t = std::vector<robot_trajectory::RobotTrajectoryPtr>;

// List of exceptions which can be thrown by the CommandListManager class.
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NegativeBlendRadiusException, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(LastBlendRadiusNotZeroException, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(StartStateSetException, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(OverlappingBlendRadiiException, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(PlanningPipelineException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(MultipleEndeffectorException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(EndeffectorWithoutLinksException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoSolverException, moveit_msgs::MoveItErrorCodes::FAILURE);

/**
 * @brief This class orchestrates the planning of single commands and
 * command lists.
 */
class CommandListManager
{
public:
  CommandListManager(const ros::NodeHandle& nh, const robot_model::RobotModelConstPtr& model);

  /**
   * @brief Generates trajectories for the specified list of motion commands.
   *
   * The folloing rules apply:
   * - If two consecutive trajectories are from the same group, they are
   * simply attached to each other, given that the blend_radius is zero.
   * - If two consecutive trajectories are from the same group, they are
   * blended together, given that the blend_radius is GREATER than zero.
   * - If two consecutive trajectories are from a different groups, then
   * the second trajectory is added as new element to the result container.
   * All following trajectories are then attached to the new trajectory
   * element (until all requests are processed or until the next group change).
   *
   * @param planning_scene The planning scene to be used for trajectory generation.
   * @param req_list List of motion requests containing: PTP, LIN, CIRC
   * and/or gripper commands.
   * Please note: A request is only valid if:
   *    - All blending radii are non negative.
   *    - The blending radius of the last request is 0.
   *    - Only the first request has a start state.
   *
   * @return Contains the calculated/generated trajectories.
   */
  RobotTrajVec_t solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                       const pilz_msgs::MotionSequenceRequest& req_list);

private:
  using MotionResponseCont = std::vector<planning_interface::MotionPlanResponse>;
  using RobotState_OptRef = boost::optional<const robot_state::RobotState& >;
  using RadiiCont = std::vector<double>;

private:
  /**
   * @brief Validates that two consecutive blending radii do not overlap.
   *
   * @param motion_plan_responses Container of calculated/generated trajectories.
   * @param radii Container stating the blend radii.
   */
  void validateBlendingRadiiDoNotOverlap(const MotionResponseCont& resp_cont,
                                         const RadiiCont &radii) const;

  /**
   * @brief Solve each sequence item individually.
   *
   * @param planning_scene The planning_scene to be used for trajectory generation.
   * @param req_list Container of requests for calculation/generation.
   *
   * @return Container of generated trajectories.
   */
  MotionResponseCont solveSequenceItems(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const pilz_msgs::MotionSequenceRequest &req_list) const;

  /**
   * @return The name of the tip frame (link) of the specified group.
   */
  const std::string& getTipFrame(const std::string& group_name) const;

  /**
   * @return TRUE if the blending radii of specified trajectories overlap,
   * otherwise FALSE. The functions returns FALSE if both trajectories are from
   * different groups.
   */
  bool checkRadiiForOverlap(const robot_trajectory::RobotTrajectory& traj_A,
                            const double radii_A,
                            const robot_trajectory::RobotTrajectory& traj_B,
                            const double radii_B) const;

private:
  /**
   * @return The last RobotState of the specified group which can
   * be found in the specified vector.
   */
  static RobotState_OptRef getPreviousEndState(const MotionResponseCont &motion_plan_responses,
                                               const std::string &group_name);

  /**
   * @brief Set start state to end state of previous calculated trajectory
   * from group.
   */
  static void setStartState(const MotionResponseCont &motion_plan_responses,
                            const std::string &group_name,
                            moveit_msgs::RobotState& start_state);


  /**
   * @return Container of radii extracted from the specified request list.
   */
  static RadiiCont getRadii(const pilz_msgs::MotionSequenceRequest &req_list);

  /**
   * @brief Validates if the request list fullfills the conditions noted
   * under pilz_trajectory_generation::CommandListManager::solve.
   */
  static void validateRequestList(const pilz_msgs::MotionSequenceRequest &req_list);

private:
  //! Node handle
  ros::NodeHandle nh_;

  //! Robot model
  moveit::core::RobotModelConstPtr model_;

  //! @brief Builder to construct the container containing the final
  //! trajectories.
  PlanComponentsBuilder plan_comp_builder_;
};

}

#endif // COMMAND_LIST_MANAGER_H
