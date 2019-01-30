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

#ifndef CIRCAUXILIARY_H
#define CIRCAUXILIARY_H

#include <moveit_msgs/Constraints.h>

namespace pilz_industrial_motion_testutils
{

/**
 * @brief Base class to define an auxiliary point needed to specify
 * circ commands.
 */
template<class AuxiliaryConfigType>
class CircAuxiliary
{
public:
  void setConfiguration(const AuxiliaryConfigType& auxiliary_config);
  const AuxiliaryConfigType& getConfiguration() const;

public:
  virtual moveit_msgs::Constraints toPathConstraints() const = 0;

protected:
  AuxiliaryConfigType auxiliary_config_;

};


template< class AuxiliaryConfigType>
void CircAuxiliary<AuxiliaryConfigType>::setConfiguration(const AuxiliaryConfigType& auxiliary_config)
{
  auxiliary_config_ = auxiliary_config;
}

template< class AuxiliaryConfigType>
inline const AuxiliaryConfigType& CircAuxiliary<AuxiliaryConfigType>::getConfiguration() const
{
  return auxiliary_config_;
}

}

#endif // CIRCAUXILIARY_H
