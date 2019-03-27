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

#ifndef TRAJECTORY_GENERATOR_PTP_H
#define TRAJECTORY_GENERATOR_PTP_H

#include "eigen3/Eigen/Eigen"
#include "pilz_trajectory_generation/trajectory_generator.h"
#include "pilz_trajectory_generation/velocity_profile_atrap.h"

namespace pilz {

//TODO date type of units

/**
 * @brief This class implements a point-to-point trajectory generator based on
 * VelocityProfile_ATrap.
 */
class TrajectoryGeneratorPTP : public TrajectoryGenerator
{
public:
  /**
   * @brief Constructor of PTP Trajectory Generator
   * @throw TrajectoryGeneratorInvalidLimitsException
   * @param model: a map of joint limits information
   */
  TrajectoryGeneratorPTP(const robot_model::RobotModelConstPtr& robot_model,
                         const pilz::LimitsContainer& planner_limits);

  ~TrajectoryGeneratorPTP();

  /**
   * @brief generate ptp robot trajectory
   * @param req: motion plan request
   * following fields are used and need to be defined properly:
   *   - start_state/joint_state/name
   *   - start_state/joint_state/position
   *   - start_state/joint_state/velocity (default 0)
   *   - goal_constraints/(joint_constraints or (position_constraints and orientation_constraints) )
   *   - group_name: name of the planning group
   *   - max_velocity_scaling_factor: scaling factor of maximal joint velocity
   *   - max_acceleration_scaling_factor: scaling factor of maximal joint acceleration/deceleration
   *
   * @param res: motion plan response
   * following fields will be provided as planning result:
   *   - trajectory_start/joint_state/(name, position and velocity)
   *   - trajectory/joint_trajectory/joint_names
   *   - trajectory/joint_trajectory/points/(positions, velocities, accelerations and time_from_start)
   *   - group_name: name of the planning group
   *   - planning_time
   *   - error_code/val
   *
   * @param sampling_time: sampling time of the generate trajectory (default 8ms)
   *
   * @return motion plan succeed/fail, detailed information in motion plan responce/error_code
   */
  virtual bool generate(const planning_interface::MotionPlanRequest& req,
                        planning_interface::MotionPlanResponse&  res,
                        double sampling_time=0.1) override;

private:

  /**
   * @brief Extract needed information from a motion plan request in order to simplify
   * further usages.
   * @param req: motion plan request
   * @param info: information extracted from motion plan request which is necessary for the planning
   * @param error_code: MoveItErrorCodes which indicates the detailed error
   * @return: ture if planning information is successfully extracted
   */
  virtual bool extractMotionPlanInfo(const planning_interface::MotionPlanRequest& req,
                                     MotionPlanInfo& info,
                                     moveit_msgs::MoveItErrorCodes& error_code) const override;

  /**
   * @brief plan ptp joint trajectory with zero start velocity
   * @param start_pos
   * @param goal_pos
   * @param joint_trajectory
   * @param group_name
   * @param velocity_scaling_factor
   * @param acceleration_scaling_factor
   * @param sampling_time
   */
  void planPTP(const std::map<std::string, double>& start_pos,
               const std::map<std::string, double>& goal_pos,
               trajectory_msgs::JointTrajectory& joint_trajectory,
               const std::string &group_name,
               const double& velocity_scaling_factor,
               const double& acceleration_scaling_factor,
               const double& sampling_time);

private:
  const double MIN_MOVEMENT = 0.001;
  pilz::JointLimitsContainer joint_limits_;
  // most strict joint limits for each group
  std::map<std::string, pilz_extensions::JointLimit> most_strict_limits_;
};

}

#endif // TRAJECTORY_GENERATOR_PTP_H
