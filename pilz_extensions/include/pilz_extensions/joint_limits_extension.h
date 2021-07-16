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

#ifndef JOINT_LIMITS_EXTENSION_H
#define JOINT_LIMITS_EXTENSION_H

#include <joint_limits_interface/joint_limits.h>
#include <map>

namespace pilz_extensions {
namespace joint_limits_interface {

/**
 * @brief Extends joint_limits_interface::JointLimits with a deceleration parameter
 */
struct JointLimits : ::joint_limits_interface::JointLimits {
  JointLimits()
    : max_deceleration(0.0),
      has_deceleration_limits(false)
  {}

/// maximum deceleration MUST(!) be negative
double max_deceleration;

bool has_deceleration_limits;

};
}

typedef joint_limits_interface::JointLimits JointLimit;
typedef std::map<std::string, JointLimit> JointLimitsMap;

}
#endif // JOINT_LIMITS_EXTENSION_H
