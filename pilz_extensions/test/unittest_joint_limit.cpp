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

#include <gtest/gtest.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include "pilz_extensions/joint_limits_extension.h"
#include "pilz_extensions/joint_limits_interface_extension.h"

namespace pilz_extensions_tests
{

class JointLimitTest : public ::testing::Test
{
};

TEST_F(JointLimitTest, SimpleRead)
{
  ros::NodeHandle node_handle("~");

  // Joints limits interface
  pilz_extensions::joint_limits_interface::JointLimits joint_limits_extended;
  joint_limits_interface::JointLimits joint_limits;

  pilz_extensions::joint_limits_interface::getJointLimits("joint_1", node_handle, joint_limits_extended);

  EXPECT_EQ(1, joint_limits_extended.max_acceleration);
  EXPECT_EQ(-1, joint_limits_extended.max_deceleration);
}

TEST_F(JointLimitTest, readNonExistingJointLimit)
{
  ros::NodeHandle node_handle("~");

  // Joints limits interface
  pilz_extensions::joint_limits_interface::JointLimits joint_limits_extended;
  joint_limits_interface::JointLimits joint_limits;

  EXPECT_FALSE(pilz_extensions::joint_limits_interface::getJointLimits("anything",
                                                                       node_handle,
                                                                       joint_limits_extended));
}

/**
 * @brief Test reading a joint limit representing an invalid parameter key
 *
 * For full coverage.
 */
TEST_F(JointLimitTest, readInvalidParameterName)
{
  ros::NodeHandle node_handle("~");

  // Joints limits interface
  pilz_extensions::joint_limits_interface::JointLimits joint_limits_extended;
  joint_limits_interface::JointLimits joint_limits;

  EXPECT_FALSE(pilz_extensions::joint_limits_interface::getJointLimits("~anything",
                                                                       node_handle,
                                                                       joint_limits_extended));
}

TEST_F(JointLimitTest, OldRead)
{
  ros::NodeHandle node_handle("~");

  // Joints limits interface
  joint_limits_interface::JointLimits joint_limits;
  joint_limits_interface::getJointLimits("joint_1", node_handle, joint_limits);

  EXPECT_EQ(1, joint_limits.max_acceleration);
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_joint_limits_extended");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
