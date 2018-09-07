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

#include "ros/ros.h"

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include "pilz_extensions/joint_limits_interface_extension.h"
#include "pilz_extensions/joint_limits_extension.h"

/**
 * Simple demo of the extensions of existing ros packages
 */
int main(int argc, char **argv)
{
  ros::init (argc, argv, "demo_extension");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // Joints limits interface
  pilz_extensions::joint_limits_interface::JointLimits joint_limits_extended;
  joint_limits_interface::JointLimits joint_limits;
  pilz_extensions::joint_limits_interface::getJointLimits("joint_1", node_handle, joint_limits_extended);

  ROS_INFO_STREAM("Acceleration Limit: " << joint_limits_extended.max_acceleration);
  ROS_INFO_STREAM("Deceleration Limit: " << joint_limits_extended.max_deceleration);



  ros::shutdown();

  return 0;
}
