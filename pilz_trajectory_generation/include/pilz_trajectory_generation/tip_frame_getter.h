#ifndef TIP_FRAME_GETTER_H
#define TIP_FRAME_GETTER_H

#include <string>
#include <cassert>

#include <moveit/robot_model/joint_model_group.h>

#include "pilz_trajectory_generation/trajectory_generation_exceptions.h"

namespace pilz_trajectory_generation
{

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(EndEffectorException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoSolverException, moveit_msgs::MoveItErrorCodes::FAILURE);

/**
 * @return The name of the tip frame (link) of the specified group. An
 * exception is thrown if the group is an end-effector or if no solver for
 * the group could be found.
 */
static const std::string& getTipFrame(const moveit::core::JointModelGroup* group)
{
  if (group->isEndEffector())
  {
    throw EndEffectorException("Given group is an end-effector which is not allowed | group: " + group->getName());
  }

  auto solver {group->getSolverInstance()};
  if(solver == nullptr)
  {
    throw NoSolverException("No solver for group " + group->getName());
  }
  return solver->getTipFrame();
}

}

#endif // TIP_FRAME_GETTER_H
