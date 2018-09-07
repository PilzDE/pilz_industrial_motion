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

#ifndef TRAJECTORY_GENERATOR_CIRC_H
#define TRAJECTORY_GENERATOR_CIRC_H

#include <eigen3/Eigen/Eigen>
#include <kdl/path.hpp>
#include <kdl/velocityprofile.hpp>
#include "pilz_trajectory_generation/trajectory_generator.h"

namespace pilz {
/**
 * @brief This class implements a trajectory generator of arcs in Cartesian space.
 * The arc is specified by a start pose, a goal pose and a interim point on the arc,
 * or a point as the center of the circle which forms the arc. Complete circle is not
 * covered by this generator.
 */
class TrajectoryGeneratorCIRC : public TrajectoryGenerator
{
public:
  /**
   * @brief Constructor of CIRC Trajectory Generator
   * @throw TrajectoryGeneratorInvalidLimitsException
   * @param model: robot model
   * @param planner_limits: limits in joint and Cartesian spaces
   */
  TrajectoryGeneratorCIRC(const robot_model::RobotModelConstPtr& robot_model,
                          const pilz::LimitsContainer& planner_limits);

  ~TrajectoryGeneratorCIRC();

  /**
   * @brief generate CIRC robot trajectory
   * @param req: motion plan request
   * following fields are used and need to be defined properly:
   *   - planner_id: CIRC
   *   - group_name: name of the planning group
   *   - max_velocity_scaling_factor: scaling factor of maximal Cartesian translational/rotational velocity
   *   - max_acceleration_scaling_factor: scaling factor of maximal Cartesian translational/rotational acceleration/deceleration
   *   - start_state/joint_state/name
   *   - start_state/joint_state/position
   *   - goal_constraints/(joint_constraints or (position_constraints and orientation_constraints) )
   *   - path_constraints/name: interim/center
   *   - path_constraints/position_constraints (link_name must be given)
   *
   * @param res: motion plan response
   * following fields will be provided as planning result:
   *   - group_name: name of the planning group
   *   - planning_time
   *   - error_code/val
   *   - trajectory_start/joint_state/(name, position and velocity)
   *   - trajectory/joint_trajectory/joint_names
   *   - trajectory/joint_trajectory/points/(positions, velocities, accelerations and time_from_start)
   *
   * @param sampling_time: sampling time of the generate trajectory (default 8ms)
   *
   * @return motion plan succeed/fail, detailed information in motion plan responce/error_code
   */
  virtual bool generate(const planning_interface::MotionPlanRequest& req,
                        planning_interface::MotionPlanResponse& res,
                        double sampling_time=0.1) override;

private:

  /**
   * @brief validate the motion plan request of CIRC motion command
   * @param req
   * @param error_code
   * @return
   */
  virtual bool validateRequest(const planning_interface::MotionPlanRequest &req,
                               moveit_msgs::MoveItErrorCodes& error_code) const final;

  /**
   * @brief extractMotionPlanInfo
   * @param req
   * @param info
   * @param error_code
   * @return
   */
  virtual bool extractMotionPlanInfo(const planning_interface::MotionPlanRequest &req,
                                     MotionPlanInfo &info,
                                     moveit_msgs::MoveItErrorCodes &error_code) const final;

  /**
   * @brief construct a KDL::Path object for a Cartesian path of an arc
   * @param req: motion plan request
   * @param error_code: moveit error code
   * @return a unique pointer of the path object. null_ptr in case of an error.
   */
  std::unique_ptr<KDL::Path> setPathCIRC(const MotionPlanInfo &info,
                                         moveit_msgs::MoveItErrorCodes &error_code) const;


};

}

#endif // TRAJECTORY_GENERATOR_CIRC_H
