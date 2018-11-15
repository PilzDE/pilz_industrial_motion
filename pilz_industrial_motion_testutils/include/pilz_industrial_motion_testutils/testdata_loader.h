/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TESTDATA_LOADER_H
#define TESTDATA_LOADER_H

#include <string>
#include <vector>
#include <utility>
#include <stdexcept>

#include <geometry_msgs/Pose.h>

namespace pilz_industrial_motion_testutils
{

/**
 * @brief Enum listing the different
 * auxiliary pose types.
 */
enum class ECircAuxPosType
{
  eINTERMEDIATE,
  eCENTER
};

struct SSequenceCmd
{
  std::string cmd_name;
  std::string cmd_type;
  double blend_radius;
};


/**
 * @brief Structure contains information of the motion command.
 * All positions are given in joint space since the tcp frame changes from model to model.
 */
struct STestMotionCommand
{
  std::string planning_group;
  std::string target_link; // all Cartesian poses refer to this link
  std::vector<double> start_position; //joint positions
  std::vector<double> start_pose; //cartesian pose, (xyz, xyzw)
  std::vector<double> goal_position; // joint positions
  std::vector<double> goal_pose; //cartesian pose, (xyz, xyzw)
  double vel_scale;
  double acc_scale;
  // only for circ
  ECircAuxPosType aux_pos_type; // center of interim points
  std::vector<double> aux_pose; //cartesian pose, (xyz, only position)
};

/**
 * @brief Abstract base class describing the interface to access
 * test data like robot poses and robot commands.
 */
class TestdataLoader
{
public:
  TestdataLoader(){}
  virtual ~TestdataLoader(){}

public:
  /**
   * @brief Returns the joint values for the given pos and group.
   *
   */
  virtual bool getJoints(const std::string &pos_name, const std::string &group_name,
                         std::vector<double> &dVec) const = 0;

  /**
   * @brief Returns the Cartesian Pose for the given pos and group.
   *
   */
  virtual bool getPose(const std::string &pos_name, const std::string &group_name,
                       std::vector<double> &dVec) const = 0;

  /**
   * @brief Get the LIN motion command structure according to the cmmand name
   * @param cmd_name
   * @param cmd
   * @return
   */
  virtual bool getLin(const std::string& cmd_name, STestMotionCommand& cmd) const = 0;

  /**
   * @brief Returns the start-, end- and auxility-position, as well as
   * the velocity and acceleration of the circ command given by its name.
   *
   * Please note: It is also necessary to state if the auxiliary point
   * of the circ command is stored as intermediate or center point.
   */
  virtual bool getCirc(const std::string &cmd_name, STestMotionCommand& cmd) const = 0;

  /**
   * @brief Returns a container containing the cmds which make-up the
   * blend cmd. The cmds in the container are in the order of execution.
   */
  virtual bool getSequence(const std::string &cmd_name,
                           std::vector<SSequenceCmd> &seq_cmds) const = 0;

public:
   static geometry_msgs::Pose fromVecToMsg(const std::vector<double>& vec);
};

inline geometry_msgs::Pose TestdataLoader::fromVecToMsg(const std::vector<double>& vec)
{
  if (vec.size() != 7)
  {
    throw std::invalid_argument("Incorrect vector size (excpected: 7).");
  }

  geometry_msgs::Pose pose;

  pose.position.x = vec[0];
  pose.position.y = vec[1];
  pose.position.z = vec[2];

  pose.orientation.x = vec[3];
  pose.orientation.y = vec[4];
  pose.orientation.z = vec[5];
  pose.orientation.w = vec[6];

  return pose;
}


}

#endif // TESTDATA_LOADER_H
