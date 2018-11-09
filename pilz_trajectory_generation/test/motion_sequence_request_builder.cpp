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

#include "motion_sequence_request_builder.h"

pilz_msgs::MotionSequenceRequest MotionSequenceRequestBuilder::build(
  std::initializer_list<std::pair<moveit_msgs::MotionPlanRequest, double> > l)
{
  pilz_msgs::MotionSequenceRequest req_list;

  for(const auto& pair : l)
  {
    pilz_msgs::MotionSequenceItem req;
    req.req = pair.first;
    req.blend_radius = pair.second;
    req_list.items.push_back(req);
  }

  return req_list;
}
