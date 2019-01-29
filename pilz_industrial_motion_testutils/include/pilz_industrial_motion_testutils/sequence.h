/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <vector>
#include <utility>

#include <pilz_msgs/MotionSequenceRequest.h>

#include "command_types_typedef.h"
#include "motioncmd.h"

namespace pilz_industrial_motion_testutils
{

/**
 * @brief Data class storing all information regarding a Sequence command.
 */
class Sequence
{
public:
  Sequence()
  {}

public:
  /**
   * @brief Adds a command to the end of the sequence.
   * @param cmd The command which has to be added.
   */
  void add(MotionCmdUPtr cmd, double radius = 0.);

  void setAllBlendRadiiToZero();

  MotionCmd& getCmd(size_t index_cmd);
  const MotionCmd& getCmd(size_t index_cmd) const;

  double getBlendRadius(size_t index_cmd) const;

  pilz_msgs::MotionSequenceRequest toRequest() const;

private:
  std::vector<std::pair<MotionCmdUPtr, double> > cmds_;
};

inline void Sequence::add(MotionCmdUPtr cmd, double radius)
{
  assert(cmd);
  cmds_.emplace_back( std::move(cmd), radius );
}

inline MotionCmd& Sequence::getCmd(size_t index_cmd)
{
  return *(cmds_.at(index_cmd).first);
}

inline const MotionCmd& Sequence::getCmd(size_t index_cmd) const
{
  return *(cmds_.at(index_cmd).first);
}

inline double Sequence::getBlendRadius(size_t index_cmd) const
{
  return cmds_.at(index_cmd).second;
}

}


#endif // SEQUENCE_H
