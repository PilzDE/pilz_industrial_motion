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

#ifndef MOTION_BLEND_REQUEST_LIST_BUILDER_H
#define MOTION_BLEND_REQUEST_LIST_BUILDER_H

#include <pilz_msgs/MotionSequenceRequest.h>

class MotionSequenceRequestBuilder
{
public:

  pilz_msgs::MotionSequenceRequest
  build(std::initializer_list<std::pair<moveit_msgs::MotionPlanRequest, double> > l);
};

#endif // MOTION_BLEND_REQUEST_LIST_BUILDER_H
