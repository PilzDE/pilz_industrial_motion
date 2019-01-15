#ifndef COMMAND_TYPES_H
#define COMMAND_TYPES_H

#include <string>

#include <moveit_msgs/MotionPlanRequest.h>

namespace pilz_industrial_motion_testutils
{

template <class StartType, class GoalType>
class BaseCommand
{
public:
  inline void setPlanningGroup(const std::string &planning_group)
  {
    planning_group_ = planning_group;
  }

  inline void setTargetLink(const std::string &target_link)
  {
    target_link_ = target_link;
  }

  inline void setVelocityScale(double velocity_scale)
  {
    vel_scale_ = velocity_scale;
  }

  inline void setAccelerationScale(double acceleration_scale)
  {
    acc_scale_ = acceleration_scale;
  }

  inline void setStartConfiguration(StartType &start)
  {
    start_ = start;
  }

  inline void setGoalConfiguration(GoalType &goal)
  {
    goal_ = goal;
  }

  inline GoalType& getStartConfiguration()
  {
    return start_;
  }

  inline GoalType& getGoalConfiguration()
  {
    return goal_;
  }

protected:
  std::string planning_group_;
  std::string target_link_; // all Cartesian poses refer to this link
  double vel_scale_;
  double acc_scale_;
  GoalType goal_;
  StartType start_;

};

template <class StartType, class GoalType>
class Ptp : public BaseCommand<StartType, GoalType>
{
public:
  Ptp()
    : BaseCommand<StartType, GoalType>()
  {}

public:
  moveit_msgs::MotionPlanRequest toRequest();

};

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <class StartType, class GoalType>
inline moveit_msgs::MotionPlanRequest Ptp<StartType, GoalType>::toRequest()
{
  moveit_msgs::MotionPlanRequest req;
  req.goal_constraints.push_back(this->goal_.toConstraints());
  req.planner_id = "PTP";

  req.max_velocity_scaling_factor = this->vel_scale_;
  req.max_acceleration_scaling_factor = this->acc_scale_;
  req.group_name = this->planning_group_;

  req.start_state = this->start_.toMoveitMsgsRobotState();

  return req;
}

}

#endif // COMMAND_TYPES_H
