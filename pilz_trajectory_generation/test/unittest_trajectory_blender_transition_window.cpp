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

#include <memory>

#include <gtest/gtest.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <eigen_conversions/eigen_msg.h>

#include <pilz_industrial_motion_testutils/xml_testdata_loader.h>
#include <pilz_industrial_motion_testutils/sequence.h>

#include "pilz_trajectory_generation/trajectory_generator_ptp.h"
#include "pilz_trajectory_generation/trajectory_generator_lin.h"
#include "pilz_trajectory_generation/trajectory_generator_circ.h"
#include "pilz_trajectory_generation/joint_limits_aggregator.h"
#include "pilz_trajectory_generation/trajectory_blender_transition_window.h"
#include "pilz_trajectory_generation/trajectory_blend_request.h"
#include "pilz_trajectory_generation/trajectory_blend_response.h"
#include "test_utils.h"

const std::string BLEND_DATA_PREFIX("test_data/");
const std::string PARAM_MODEL_NO_GRIPPER_NAME {"robot_description"};
const std::string PARAM_MODEL_WITH_GRIPPER_NAME {"robot_description_pg70"};

//parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");
const std::string CARTESIAN_VELOCITY_TOLERANCE("cartesian_velocity_tolerance");
const std::string CARTESIAN_ANGULAR_VELOCITY_TOLERANCE("cartesian_angular_velocity_tolerance");
const std::string JOINT_VELOCITY_TOLERANCE("joint_velocity_tolerance");
const std::string JOINT_ACCELERATION_TOLERANCE("joint_acceleration_tolerance");
const std::string OTHER_TOLERANCE("other_tolerance");
const std::string SAMPLING_TIME("sampling_time");
const std::string BLEND_DATASET_NUM("blend_dataset_num");
const std::string TEST_DATA_FILE_NAME("testdata_file_name");

using namespace pilz;
using namespace pilz_industrial_motion_testutils;

class TrajectoryBlenderTransitionWindowTest: public testing::TestWithParam<std::string>
{
protected:

  /**
   * @brief Create test scenario for trajectory blender
   *
   */
  virtual void SetUp();

  planning_interface::MotionPlanResponse generateLinTraj(const Sequence& seq, size_t index);

protected:
  // ros stuff
  ros::NodeHandle ph_ {"~"};
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader(GetParam()).getModel()};

  std::shared_ptr<TrajectoryGenerator> lin_generator_;
  std::unique_ptr<TrajectoryBlenderTransitionWindow> blender_;

  // test parameters from parameter server
  std::string planning_group_, target_link_;
  double cartesian_velocity_tolerance_, cartesian_angular_velocity_tolerance_,
  joint_velocity_tolerance_, joint_acceleration_tolerance_, other_tolerance_, sampling_time_;
  std::vector<testutils::blend_test_data> test_data_;
  LimitsContainer planner_limits_;

  std::string test_data_file_name_;
  XmlTestDataLoaderUPtr data_loader_;

};

void TrajectoryBlenderTransitionWindowTest::SetUp()
{
  // get parameters
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));
  ASSERT_TRUE(ph_.getParam(CARTESIAN_VELOCITY_TOLERANCE, cartesian_velocity_tolerance_));
  ASSERT_TRUE(ph_.getParam(CARTESIAN_ANGULAR_VELOCITY_TOLERANCE, cartesian_angular_velocity_tolerance_));
  ASSERT_TRUE(ph_.getParam(JOINT_VELOCITY_TOLERANCE, joint_velocity_tolerance_));
  ASSERT_TRUE(ph_.getParam(JOINT_ACCELERATION_TOLERANCE, joint_acceleration_tolerance_));
  ASSERT_TRUE(ph_.getParam(OTHER_TOLERANCE, other_tolerance_));
  ASSERT_TRUE(ph_.getParam(SAMPLING_TIME, sampling_time_));
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));

  // load the test data provider
  data_loader_.reset(new XmlTestdataLoader(test_data_file_name_, robot_model_));
  ASSERT_NE(nullptr, data_loader_) << "Failed to load test data by provider.";

  // check robot model
  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

  // create the limits container
  pilz::JointLimitsContainer joint_limits =
      pilz::JointLimitsAggregator::getAggregatedLimits(ph_, robot_model_->getActiveJointModels());
  CartesianLimit cart_limits;
  cart_limits.setMaxRotationalVelocity(1*M_PI);
  cart_limits.setMaxTranslationalAcceleration(2);
  cart_limits.setMaxTranslationalDeceleration(2);
  cart_limits.setMaxTranslationalVelocity(1);
  planner_limits_.setJointLimits(joint_limits);
  planner_limits_.setCartesianLimits(cart_limits);

  // initialize trajectory generators and blender
  lin_generator_.reset(new TrajectoryGeneratorLIN(robot_model_, planner_limits_));
  ASSERT_NE(nullptr, lin_generator_) << "failed to create LIN trajectory generator";
  blender_.reset(new TrajectoryBlenderTransitionWindow(planner_limits_));
  ASSERT_NE(nullptr, blender_) << "failed to create trajectory blender";

  // get the test data set
  int blend_dataset_num;
  ASSERT_TRUE(ph_.getParam(BLEND_DATASET_NUM, blend_dataset_num));
  ASSERT_TRUE(testutils::getBlendTestData(ph_, blend_dataset_num, BLEND_DATA_PREFIX, test_data_));
}


planning_interface::MotionPlanResponse TrajectoryBlenderTransitionWindowTest::generateLinTraj(const Sequence &seq, size_t index)
{
  planning_interface::MotionPlanRequest req {seq.getCmd(index).toRequest()};
  planning_interface::MotionPlanResponse resp;
  if(!lin_generator_->generate(req, resp, sampling_time_))
  {
    std::runtime_error("Failed to generate trajectory.");
  }
  return resp;
}

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_CASE_P(InstantiationName, TrajectoryBlenderTransitionWindowTest, ::testing::Values(
                          PARAM_MODEL_NO_GRIPPER_NAME,
                          PARAM_MODEL_WITH_GRIPPER_NAME
                          ));


/**
 * @brief  Tests the blending of two trajectories with an invalid group name.
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories.
 *    2. Try to generate blending trajectory with invalid group name.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, invalidGroupName)
{
  auto test_data = test_data_.front();
  // generate two lin trajectories from test dataset
  planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
  double dis_lin_1, dis_lin_2;
  ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                       lin_,
                                                       planning_group_,
                                                       target_link_,
                                                       test_data,
                                                       sampling_time_, sampling_time_,
                                                       res_lin_1, res_lin_2,
                                                       dis_lin_1, dis_lin_2))
      << "Failed to generate LIN trajectories from test data";

  // Generate blend trajectory
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = "invalid_group_name";
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;

  // select blend radius
  blend_req.blend_radius = dis_lin_1 > dis_lin_2 ? 0.5*dis_lin_2 : 0.5*dis_lin_1;
  EXPECT_FALSE(blender_->blend(blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with an invalid target link.
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories.
 *    2. Try to generate blending trajectory with invalid target link.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, invalidTargetLink)
{
  auto test_data = test_data_.front();
  // generate two lin trajectories from test dataset
  planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
  double dis_lin_1, dis_lin_2;
  ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                       lin_,
                                                       planning_group_,
                                                       target_link_,
                                                       test_data,
                                                       sampling_time_, sampling_time_,
                                                       res_lin_1, res_lin_2,
                                                       dis_lin_1, dis_lin_2))
      << "Failed to generate LIN trajectories from test data";

  // Generate blend trajectory
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = "invalid_target_link";
  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;

  // select blend radius
  blend_req.blend_radius = dis_lin_1 > dis_lin_2 ? 0.5*dis_lin_2 : 0.5*dis_lin_1;
  EXPECT_FALSE(blender_->blend(blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with a negative blending radius.
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories.
 *    2. Try to generate blending trajectory with negative blending radius.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, negativeRadius)
{
  auto test_data = test_data_.front();
  // generate two lin trajectories from test dataset
  planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
  double dis_lin_1, dis_lin_2;
  ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                       lin_generator_,
                                                       planning_group_,
                                                       target_link_,
                                                       test_data,
                                                       sampling_time_, sampling_time_,
                                                       res_lin_1, res_lin_2,
                                                       dis_lin_1, dis_lin_2))
      << "Failed to generate LIN trajectories from test data";

  // Generate blend trajectory
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;

  // select blend radius
  blend_req.blend_radius = -0.1;
  EXPECT_FALSE(blender_->blend(blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with zero blending radius.
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories.
 *    2. Try to generate blending trajectory with zero blending radius.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, zeroRadius)
{
  auto test_data = test_data_.front();
  // generate two lin trajectories from test dataset
  planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
  double dis_lin_1, dis_lin_2;
  ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                       lin_generator_,
                                                       planning_group_,
                                                       target_link_,
                                                       test_data,
                                                       sampling_time_, sampling_time_,
                                                       res_lin_1, res_lin_2,
                                                       dis_lin_1, dis_lin_2))
      << "Failed to generate LIN trajectories from test data";

  // Generate blend trajectory
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;

  // select blend radius
  blend_req.blend_radius = 0.0;
  EXPECT_FALSE(blender_->blend(blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with differenent sampling times.
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories with different sampling times.
 *    2. Try to generate blending trajectory.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, differentSamplingTimes)
{
  auto test_data = test_data_.front();
  // generate two lin trajectories from test dataset
  planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
  double dis_lin_1, dis_lin_2;
  ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                       lin_generator_,
                                                       planning_group_,
                                                       target_link_,
                                                       test_data,
                                                       sampling_time_, 2*sampling_time_,
                                                       res_lin_1, res_lin_2,
                                                       dis_lin_1, dis_lin_2))
      << "Failed to generate LIN trajectories from test data";

  // Generate blend trajectory
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;
  blend_req.blend_radius = dis_lin_1 > dis_lin_2 ? 0.5*dis_lin_2 : 0.5*dis_lin_1;
  EXPECT_FALSE(blender_->blend(blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with one trajectory having non-uniform sampling time (apart from the
 * last sample, which is ignored).
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories and corrupt uniformity of sampling time.
 *    2. Try to generate blending trajectory.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, nonUniformSamplingTime)
{
  auto test_data = test_data_.front();
  // generate two lin trajectories from test dataset
  planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
  double dis_lin_1, dis_lin_2;
  ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                       lin_generator_,
                                                       planning_group_,
                                                       target_link_,
                                                       test_data,
                                                       sampling_time_, sampling_time_,
                                                       res_lin_1, res_lin_2,
                                                       dis_lin_1, dis_lin_2))
      << "Failed to generate LIN trajectories from test data";

  // Modify first time interval
  EXPECT_GT(res_lin_1.trajectory_->getWayPointCount(), 2u);
  res_lin_1.trajectory_->setWayPointDurationFromPrevious(1, 2*sampling_time_);

  // Generate blend trajectory
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;
  blend_req.blend_radius = dis_lin_1 > dis_lin_2 ? 0.5*dis_lin_2 : 0.5*dis_lin_1;
  EXPECT_FALSE(blender_->blend(blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two linear trajectories which do not intersect.
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories from valid test data set
 *    2. Replace the second trajectory by the first one
 *    2. Try to generate blending trajectory.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Two LIN do not intersect.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, notIntersectingLinTrajectories)
{
  auto test_data = test_data_.front();
  // generate two lin trajectories from test dataset
  planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
  double dis_lin_1, dis_lin_2;
  ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                       lin_generator_,
                                                       planning_group_,
                                                       target_link_,
                                                       test_data,
                                                       sampling_time_, sampling_time_,
                                                       res_lin_1, res_lin_2,
                                                       dis_lin_1, dis_lin_2))
      << "Failed to generate LIN trajectories from test data";

  // replace the second trajectory to make the two LIN trajectories timely not intersect
  res_lin_2.trajectory_ = res_lin_1.trajectory_;

  // try to blend the two LIN trajectories
  // Generate blend trajectory
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;
  blend_req.blend_radius = dis_lin_1>dis_lin_2 ? 0.5*dis_lin_2 : 0.5*dis_lin_1;
  EXPECT_FALSE(blender_->blend(blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two cartesian linear trajectories with the
 * shared point (last point of first, first point of second trajectory) having a non-zero velocity
 *
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories from the test data set.
 *    2. Generate blending trajectory modify the shared point to have velocity.
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */

TEST_P(TrajectoryBlenderTransitionWindowTest, testNonStationaryPoint)
{
  auto test_data = test_data_.front();
  // generate two lin trajectories
  planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
  double dis_lin_1, dis_lin_2;
  ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                       lin_generator_,
                                                       planning_group_,
                                                       target_link_,
                                                       test_data,
                                                       sampling_time_, sampling_time_,
                                                       res_lin_1, res_lin_2,
                                                       dis_lin_1, dis_lin_2))
      << "Failed to generate LIN trajectories from test data";

  // blend two lin trajectories and check the result
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.blend_radius = dis_lin_1 > dis_lin_2 ? 0.5*dis_lin_2 : 0.5*dis_lin_1;

  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;

  // Modify last waypoint of first trajectory and first point of second trajectory
  blend_req.first_trajectory->getLastWayPointPtr()->setVariableVelocity(0, 1.0);
  blend_req.second_trajectory->getFirstWayPointPtr()->setVariableVelocity(0, 1.0);

  EXPECT_FALSE(blender_->blend(blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two cartesian linear trajectories where one trajectory is completely within the
 *         sphere defined by the blend radius
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories from the test data set.
 *    2. Generate blending trajectory with a blend_radius larger than the smaller trajectory.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testLINLINBlendingTrajectoryInsideBlendRadius)
{
  // Use only subset of testdata
  std::vector<testutils::blend_test_data> test_data_set;
  test_data_set.push_back(test_data_[0]);
  test_data_set.push_back(test_data_[1]);

  for(auto const &test_data : test_data_set)
  {
    // generate two lin trajectories
    planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
    double dis_lin_1, dis_lin_2;
    ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                         lin_generator_,
                                                         planning_group_,
                                                         target_link_,
                                                         test_data,
                                                         sampling_time_, sampling_time_,
                                                         res_lin_1, res_lin_2,
                                                         dis_lin_1, dis_lin_2))
        << "Failed to generate LIN trajectories from test data";

    // blend two lin trajectories and check the result
    pilz::TrajectoryBlendRequest blend_req;
    pilz::TrajectoryBlendResponse blend_res;

    blend_req.group_name = planning_group_;
    blend_req.link_name = target_link_;
    blend_req.blend_radius = dis_lin_1 > dis_lin_2 ? 1.1*dis_lin_2 : 1.1*dis_lin_1;

    blend_req.first_trajectory = res_lin_1.trajectory_;
    blend_req.second_trajectory = res_lin_2.trajectory_;

    EXPECT_FALSE(blender_->blend(blend_req, blend_res));
  }
}

/**
 * @brief  Tests the blending of two cartesian linear trajectories using robot model
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories from the test data set.
 *    2. Generate blending trajectory.
 *    3. Check blending trajectory:
 *      - for position, velocity, and acceleration bounds,
 *      - for continuity in joint space,
 *      - for continuity in cartesian space.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory generated.
 *    3. No bound is violated, the trajectories are continuous
 *        in joint and cartesian space.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testLINLINBlending)
{
  Sequence seq {data_loader_->getSequence("TestBlend")};

  planning_interface::MotionPlanResponse res_lin_1 {generateLinTraj(seq, 0)};
  planning_interface::MotionPlanResponse res_lin_2 {generateLinTraj(seq, 1)};

  // blend two lin trajectories and check the result
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.blend_radius = seq.getBlendRadius(0);

  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;

  EXPECT_TRUE(blender_->blend(blend_req, blend_res));

  EXPECT_TRUE(testutils::checkBlendResult(blend_req,
                                          blend_res,
                                          planner_limits_,
                                          joint_velocity_tolerance_,
                                          joint_acceleration_tolerance_,
                                          cartesian_velocity_tolerance_,
                                          cartesian_angular_velocity_tolerance_));

}

/**
 * @brief  Tests the blending of two cartesian linear trajectories with a non-trivial overlap using robot model
 * The test sequence is repeated twice, using robot model with and without gripper
 * Test Sequence:
 *    1. Generate two linear trajectories from the test data set, such that the second one is the reversed first one.
 *    2. Generate blending trajectory.
 *    3. Check blending trajectory:
 *      - for position, velocity, and acceleration bounds,
 *      - for continuity in joint space,
 *      - for continuity in cartesian space.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory generated.
 *    3. No bound is violated, the trajectories are continuous
 *        in joint and cartesian space.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testLINLINBlendingOverlappingTrajectories)
{
  auto test_data = test_data_.front();

  // modify end position such that trajectories overlap
  test_data.end_position = test_data.start_position;

  // generate two lin trajectories from test dataset
  planning_interface::MotionPlanResponse res_lin_1, res_lin_2;
  double dis_lin_1, dis_lin_2;
  ASSERT_TRUE(testutils::generateTrajFromBlendTestData(robot_model_,
                                                       lin_,
                                                       planning_group_,
                                                       target_link_,
                                                       test_data,
                                                       sampling_time_, sampling_time_,
                                                       res_lin_1, res_lin_2,
                                                       dis_lin_1, dis_lin_2))
      << "Failed to generate LIN trajectories from test data";

  // blend two lin trajectories and check the result
  pilz::TrajectoryBlendRequest blend_req;
  pilz::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res_lin_1.trajectory_;
  blend_req.second_trajectory = res_lin_2.trajectory_;
  blend_req.blend_radius = 0.5*dis_lin_1;
  EXPECT_TRUE(blender_->blend(blend_req, blend_res));

  // check the blend result
  EXPECT_TRUE(testutils::checkBlendResult(blend_req,
                                          blend_res,
                                          planner_limits_,
                                          joint_velocity_tolerance_,
                                          joint_acceleration_tolerance_,
                                          cartesian_velocity_tolerance_,
                                          cartesian_angular_velocity_tolerance_));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_trajectory_blender_transition_window");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
