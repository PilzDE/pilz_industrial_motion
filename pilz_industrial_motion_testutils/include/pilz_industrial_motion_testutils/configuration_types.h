#ifndef CONFIGURATION_TYPES_H
#define CONFIGURATION_TYPES_H

#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>

namespace pilz_industrial_motion_testutils
{

class RobotConfiguration
{
  virtual moveit_msgs::Constraints toConstraints() const = 0;
  virtual moveit_msgs::RobotState toMoveitMsgsRobotState() const = 0;
};

class JointConfiguration : public RobotConfiguration
{
public:

  JointConfiguration()
    : RobotConfiguration()
  {}

  JointConfiguration(const std::vector<double>& config)
    : RobotConfiguration()
    , joints_(config)
  {}

public:
  void setJointPrefix(const std::string& joint_prefix)
  {
    joint_prefix_ = joint_prefix;
  }

  moveit_msgs::Constraints toConstraints() const override;
  moveit_msgs::RobotState toMoveitMsgsRobotState() const override;

private:
  std::vector<double> joints_; // joint positions
  std::string joint_prefix_ {};
};

class CartesianConfiguration : public RobotConfiguration
{
public:
  moveit_msgs::Constraints toConstraints() const override;
  moveit_msgs::RobotState toMoveitMsgsRobotState() const override;

private:
  std::vector<double> pose_; // joint positions
};

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// TODO NOT OPTIMAL!
inline moveit_msgs::Constraints JointConfiguration::toConstraints() const
{
  moveit_msgs::Constraints gc; // TODO Code duplication test_utils.h generateJointConstraint

  auto pos_it = joints_.begin();

  for(size_t i = 0; i < joints_.size(); ++i)
  {
    moveit_msgs::JointConstraint jc;
    jc.joint_name = joint_prefix_ + std::to_string(i+1);
    jc.position = *pos_it;
    gc.joint_constraints.push_back(jc);

    ++pos_it;
  }

  return gc;
}

// TODO NOT OPTIMAL!
inline moveit_msgs::RobotState JointConfiguration::toMoveitMsgsRobotState() const
{
  moveit_msgs::RobotState robot_state;

  auto posit = joints_.begin();
  size_t i = 0;

  while(posit != joints_.end())
  {
    robot_state.joint_state.name.push_back(joint_prefix_ + std::to_string(i+1)); // TODO Code duplication getJointName
    robot_state.joint_state.position.push_back(*posit);

    i++;
    posit++;
  }
  return robot_state;
}

inline moveit_msgs::Constraints CartesianConfiguration::toConstraints() const
{
  moveit_msgs::Constraints gc;
  return gc;
}

inline moveit_msgs::RobotState CartesianConfiguration::toMoveitMsgsRobotState() const
{
  moveit_msgs::RobotState robot_state;

  // TODO

  return robot_state;
}

}

#endif // CONFIGURATION_TYPES_H
