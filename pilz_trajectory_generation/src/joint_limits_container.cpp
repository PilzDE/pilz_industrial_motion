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

#include "pilz_trajectory_generation/joint_limits_container.h"

#include "ros/ros.h"
#include <stdexcept>

bool pilz::JointLimitsContainer::addLimit(const std::string &joint_name, pilz_extensions::JointLimit joint_limit)
{
  if(joint_limit.has_deceleration_limits && joint_limit.max_deceleration >= 0)
  {
    ROS_ERROR_STREAM("joint_limit.max_deceleration MUST be negative!");
    return false;
  }
  else
  {
    container_.insert(std::pair<std::string, pilz_extensions::JointLimit>(joint_name, joint_limit));
    return true;
  }

}

bool pilz::JointLimitsContainer::hasLimit(const std::string &joint_name) const
{
  return container_.find(joint_name) != container_.end();
}

size_t pilz::JointLimitsContainer::getCount() const
{
  return container_.size();
}

bool pilz::JointLimitsContainer::empty() const
{
  return container_.empty();
}

pilz_extensions::JointLimit pilz::JointLimitsContainer::getCommonLimit() const
{
  pilz_extensions::JointLimit common_limit;

  if(container_.empty())
  {
    common_limit.has_position_limits = false;
    common_limit.min_position = 0;
    common_limit.max_position = 0;
    common_limit.has_velocity_limits = false;
    common_limit.max_velocity = 0;
    common_limit.has_acceleration_limits = false;
    common_limit.max_acceleration = 0;
    common_limit.has_deceleration_limits = false;
    common_limit.max_deceleration = 0;
    return common_limit;
  }

  common_limit = container_.begin()->second;

  for(auto it = std::next(container_.begin()); it != container_.end(); ++it)
  {

    // If this limit has position limits
    if(it->second.has_position_limits)
    {
      // Merge if common_limit has allready limit
      if(common_limit.has_position_limits)
      {
        common_limit.min_position = std::max(common_limit.min_position, it->second.min_position);
        common_limit.max_position = std::min(common_limit.max_position, it->second.max_position);
      }
      else
      {
        common_limit.has_position_limits = true;
        common_limit.min_position = it->second.min_position;
        common_limit.max_position = it->second.max_position;
      }
    }

    // If this limit has velocity limits
    if(it->second.has_velocity_limits)
    {
      // Merge if common_limit has allready limit
      if(common_limit.has_velocity_limits)
      {
        common_limit.max_velocity = std::min(common_limit.max_velocity, it->second.max_velocity);
      }
      else
      {
        common_limit.has_velocity_limits = true;
        common_limit.max_velocity = it->second.max_velocity;
      }
    }

    // If this limit has acceleration limits
    if(it->second.has_acceleration_limits)
    {
      // Merge if common_limit has allready limit
      if(common_limit.has_acceleration_limits)
      {
        common_limit.max_acceleration = std::min(common_limit.max_acceleration, it->second.max_acceleration);
      }
      else
      {
        common_limit.has_acceleration_limits = true;
        common_limit.max_acceleration = it->second.max_acceleration;
      }
    }

    // If this limit has deceleration limits
    if(it->second.has_deceleration_limits)
    {
      // Merge if common_limit has allready limit
      if(common_limit.has_deceleration_limits)
      {
        common_limit.max_deceleration = std::max(common_limit.max_deceleration, it->second.max_deceleration);
      }
      else
      {
        common_limit.has_deceleration_limits = true;
        common_limit.max_deceleration = it->second.max_deceleration;
      }
    }
  }

  return common_limit;
}

pilz_extensions::JointLimit pilz::JointLimitsContainer::getLimit(const std::string &joint_name) const
{
  return container_.at(joint_name);
}

std::map<std::string, pilz_extensions::JointLimit>::const_iterator pilz::JointLimitsContainer::begin() const
{
  return container_.begin();
}

std::map<std::string, pilz_extensions::JointLimit>::const_iterator pilz::JointLimitsContainer::end() const
{
  return container_.end();
}

bool pilz::JointLimitsContainer::verifyVelocityLimit(const std::string &joint_name,
                                                     const double &joint_velocity) const
{
  return (!(hasLimit(joint_name)
          && getLimit(joint_name).has_velocity_limits
          && fabs(joint_velocity) > getLimit(joint_name).max_velocity));
}


bool pilz::JointLimitsContainer::verifyPositionLimit(const std::string &joint_name,
                                                     const double &joint_position) const
{
  return (!( hasLimit(joint_name)
             && getLimit(joint_name).has_position_limits
             && (joint_position < getLimit(joint_name).min_position
                || joint_position > getLimit(joint_name).max_position) ) );
}


bool pilz::JointLimitsContainer::verifyPositionLimits(const std::vector<std::string> &joint_names,
                                                    const std::vector<double> &joint_positions) const
{
  if(joint_names.size() != joint_positions.size())
  {
    throw std::out_of_range("joint_names vector has a different size than joint_positions vector.");
  }

  for(std::size_t i=0; i<joint_names.size(); ++i)
  {
    if(!verifyPositionLimit(joint_names.at(i), joint_positions.at(i)))
    {
      return false;
    }
  }

  return true;
}
