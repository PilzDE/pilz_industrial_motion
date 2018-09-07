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

#ifndef TRAJECTORY_GENERATOR_LIN_H
#define TRAJECTORY_GENERATOR_LIN_H

#include "eigen3/Eigen/Eigen"
#include "pilz_trajectory_generation/trajectory_generator.h"
#include "pilz_trajectory_generation/velocity_profile_atrap.h"
#include <kdl/rotational_interpolation_sa.hpp>

namespace pilz {

//TODO date type of units

/**
 * @brief This class implements a linear trajectory generator in Cartesian space.
 * The Cartesian trajetory are based on trapezoid velocity profile.
 */
class TrajectoryGeneratorLIN : public TrajectoryGenerator
{
public:
  /**
   * @brief Constructor of LIN Trajectory Generator
   * @throw TrajectoryGeneratorInvalidLimitsException
   * @param model: robot model
   * @param planner_limits: limits in joint and Cartesian spaces
   */
  TrajectoryGeneratorLIN(const robot_model::RobotModelConstPtr& robot_model,
                         const pilz::LimitsContainer& planner_limits);

  ~TrajectoryGeneratorLIN();

  /**
   * @brief generate LIN robot trajectory
   * @param req: motion plan request
   * following fields are used and need to be defined properly:
   *   - start_state/joint_state/name
   *   - start_state/joint_state/position
   *   - goal_constraints/(joint_constraints or (position_constraints and orientation_constraints) )
   *   - group_name: name of the planning group
   *   - max_velocity_scaling_factor: scaling factor of maximal Cartesian translational/rotational velocity
   *   - max_acceleration_scaling_factor: scaling factor of maximal Cartesian translational/rotational acceleration/deceleration
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
   * @param sampling_time: sampling time of the generate trajectory (default 100ms)
   *
   * @return motion plan succeed/fail, detailed information in motion plan responce/error_code
   */
  virtual bool generate(const planning_interface::MotionPlanRequest& req,
                        planning_interface::MotionPlanResponse& res,
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
                                     moveit_msgs::MoveItErrorCodes& error_code) const final;

  /**
   * @brief construct a KDL::Path object for a Cartesian straight line
   * @param req: motion plan request
   * @param error_code: moveit error code
   * @return a unique pointer of the path object. null_ptr in case of an error.
   */
  std::unique_ptr<KDL::Path> setPathLIN(const MotionPlanInfo &info,
                                        moveit_msgs::MoveItErrorCodes &error_code) const;

};

}

#endif // TRAJECTORY_GENERATOR_LIN_H
