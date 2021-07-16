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

#ifndef JOINT_LIMITS_INTERFACE_EXTENSION_H
#define JOINT_LIMITS_INTERFACE_EXTENSION_H

#include <joint_limits_interface/joint_limits_rosparam.h>
#include "pilz_extensions/joint_limits_extension.h"

namespace pilz_extensions {
namespace joint_limits_interface {

/**
 * @see joint_limits_inteface::getJointLimits(...)
 */
inline bool getJointLimits(const std::string& joint_name,
                           const ros::NodeHandle& nh,
                           ::pilz_extensions::joint_limits_interface::JointLimits& limits) {

  // Node handle scoped where the joint limits are
  // defined (copied from ::joint_limits_interface::getJointLimits(joint_name, nh, limits)
  ros::NodeHandle limits_nh;
  try
  {
    const std::string limits_namespace = "joint_limits/" + joint_name;
    if (!nh.hasParam(limits_namespace))
    {
      ROS_DEBUG_STREAM("No joint limits specification found for joint '" << joint_name <<
                       "' in the parameter server (namespace " << nh.getNamespace() + "/" + limits_namespace << ").");
      return false;
    }
    limits_nh = ros::NodeHandle(nh, limits_namespace);
  }
  catch(const ros::InvalidNameException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  // Set the existing limits
  if(! ::joint_limits_interface::getJointLimits(joint_name, nh, limits)) {
    return false; //LCOV_EXCL_LINE // The case where getJointLimits returns false is covered above.
  }

  // Deceleration limits
  bool has_deceleration_limits = false;
  if(limits_nh.getParam("has_deceleration_limits", has_deceleration_limits))
  {
    if (!has_deceleration_limits) {limits.has_deceleration_limits = false;}
    double max_dec;
    if (has_deceleration_limits && limits_nh.getParam("max_deceleration", max_dec))
    {
      limits.has_deceleration_limits = true;
      limits.max_deceleration = max_dec;
    }
  }

  return true;
}

}
}

#endif // JOINT_LIMITS_INTERFACE_EXTENSION_H
