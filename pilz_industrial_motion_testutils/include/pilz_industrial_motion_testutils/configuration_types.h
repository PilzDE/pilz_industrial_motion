#ifndef CONFIGURATION_TYPES_H
#define CONFIGURATION_TYPES_H

#include <string>
#include <vector>
#include <cassert>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
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
  {
  }


public:
  inline JointConfiguration& setJointPrefix(const std::string& joint_prefix)
  {
    joint_prefix_ = joint_prefix;
    return *this;
  }

  moveit_msgs::Constraints toConstraints() const override;
  moveit_msgs::RobotState toMoveitMsgsRobotState() const override;
  sensor_msgs::JointState toSensorMsg() const;

private:
  inline static std::string createJointName(const std::string& joint_prefix,
                                            const size_t& joint_number)
  {
    return joint_prefix + std::to_string(joint_number);
  }

private:
  //! Joint positions
  std::vector<double> joints_;
  std::string joint_prefix_ {};
};

class CartesianConfiguration : public RobotConfiguration
{
public:
  CartesianConfiguration()
    : RobotConfiguration()
  {}

  CartesianConfiguration(const std::vector<double>& config)
    : RobotConfiguration()
    , pose_(config)
  {
  }

public:
  moveit_msgs::Constraints toConstraints() const override;
  moveit_msgs::RobotState toMoveitMsgsRobotState() const override;

private:
  std::vector<double> pose_;
};

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// TODO NOT OPTIMAL!
inline moveit_msgs::Constraints JointConfiguration::toConstraints() const
{
  moveit_msgs::Constraints gc;

  for(size_t i = 0; i < joints_.size(); ++i)
  {
    moveit_msgs::JointConstraint jc;
    jc.joint_name = createJointName(joint_prefix_, i+1);
    jc.position = joints_.at(i);
    gc.joint_constraints.push_back(jc);
  }

  return gc;
}

// TODO NOT OPTIMAL!
inline moveit_msgs::RobotState JointConfiguration::toMoveitMsgsRobotState() const
{
  moveit_msgs::RobotState robot_state;
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    robot_state.joint_state.name.emplace_back( createJointName(joint_prefix_, i+1) );
    robot_state.joint_state.position.push_back(joints_.at(i));
  }
  return robot_state;
}

// TODO Inlining NOT OPTIMAL!
inline sensor_msgs::JointState JointConfiguration::toSensorMsg() const
{
  sensor_msgs::JointState state;
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    state.name.emplace_back( createJointName(joint_prefix_, i+1) );
    state.position.push_back(joints_.at(i));
  }

  return state;
}

inline moveit_msgs::Constraints CartesianConfiguration::toConstraints() const
{
  moveit_msgs::Constraints gc;

  // TODO Add implementation

  return gc;
}

inline moveit_msgs::RobotState CartesianConfiguration::toMoveitMsgsRobotState() const
{
  moveit_msgs::RobotState robot_state;

  // TODO Add implementation

  return robot_state;
}

}

#endif // CONFIGURATION_TYPES_H
