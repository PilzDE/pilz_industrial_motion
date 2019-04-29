#ifndef TIP_FRAME_GETTER_H
#define TIP_FRAME_GETTER_H

#include <string>
#include <cassert>
#include <stdexcept>

#include <moveit/robot_model/joint_model_group.h>

#include "pilz_trajectory_generation/trajectory_generation_exceptions.h"

namespace pilz_trajectory_generation
{

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoSolverException, moveit_msgs::MoveItErrorCodes::FAILURE);

/**
 * @returns true if the JointModelGroup has a solver, false otherwise.
 *
 * @throws exception in case group is null.
 */
static bool hasSolver(const moveit::core::JointModelGroup* group)
{
  if (group == nullptr)
  {
    throw std::invalid_argument("Group must not be null");
  }
  return group->getSolverInstance() != nullptr;
}

/**
 * @return The name of the tip frame (link) of the specified group
 * returned by the solver. An exception is thrown if no solver for
 * the group could be found.
 *
 * @throws exception in case the solver has no solver.
 */
static const std::string& getSolverTipFrame(const moveit::core::JointModelGroup* group)
{
  if( !hasSolver(group) )
  {
    throw NoSolverException("No solver for group " + group->getName());
  }
  return group->getSolverInstance()->getTipFrame();
}

}

#endif // TIP_FRAME_GETTER_H
