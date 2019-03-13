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

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NegativeBlendRadiusException, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(LastBlendRadiusNotZeroException, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(StartStateSetException, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(OverlappingBlendRadiiException, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(PlanningPipelineException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(MultipleEndeffectorException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(EndeffectorWithoutLinksException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoSolverException, moveit_msgs::MoveItErrorCodes::FAILURE);

/**
 * @brief The CommandListManager class
 * This class can create a smooth trajectory from a given list of motion commands.
 * The trajectory generated from the motion commands are blended with each other within a blend radius given
 * within the MotionSequenceRequest.
 */
class CommandListManager
{
public:
  /**
   * @brief CommandListManager
   * @param model The robot model
   */
  CommandListManager(const ros::NodeHandle& nh, const robot_model::RobotModelConstPtr& model);

  // TODO: Update documentation
  /**
   * @brief Returns a full trajectory consistenting of planned trajectory
   * blended with each other in the given blend_radius.
   *
   * @param planning_scene The current planning scene.
   * @param req_list List of motion requests. Contains PTP, LIN and CIRC commands.
   *        A request is valid if:
   *        - All request are about the same group
   *        - All blending radii are non negative
   *        - The blending radius of the last request is 0
   *        - Only the first request has a start state

   * @return The resulting trajectories.
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
   * @param motion_plan_responses Container of responses from the trajectory generator.
   * @param radii Container with the blending radii
   */
  void validateBlendingRadiiDoNotOverlap(const MotionResponseCont& resp_cont,
                                         const RadiiCont &radii);

  /**
   * @brief Solve each sequence item individually.
   *
   * @param planning_scene The planning_scene
   * @param req_list The motion plan request list
   *
   * @return Constains the generated trajectories.
   */
  MotionResponseCont solveSequenceItems(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const pilz_msgs::MotionSequenceRequest &req_list);

  /**
   * @brief The the name of the to frame (link) of the given group
   * @return name as string
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
   * @brief Returns the last RobotState of the specified group which can
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


  static RadiiCont getRadii(const pilz_msgs::MotionSequenceRequest &req_list);

  /**
   * @brief Validate if the request list fullfills the conditions noted
   *        under pilz_trajectory_generation::CommandListManager::solve
   */
  static void validateRequestList(const pilz_msgs::MotionSequenceRequest &req_list);

private:
  /// Node handle
  ros::NodeHandle nh_;

  /// Robot model
  moveit::core::RobotModelConstPtr model_;

  PlanComponentsBuilder plan_comp_builder_;
};

}

#endif // COMMAND_LIST_MANAGER_H
