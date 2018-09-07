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

#include "pilz_trajectory_generation/path_circle_generator.h"

namespace pilz {

std::unique_ptr<KDL::Path> PathCircleGenerator::circleFromCenter(
    const KDL::Frame &start_pose,
    const KDL::Frame &goal_pose,
    const KDL::Vector &center_point,
    double eqradius
    )
{
  double a = (start_pose.p - center_point).Norm();
  double b = (goal_pose.p - center_point).Norm();
  double c = (start_pose.p - goal_pose.p).Norm();

  if(fabs(a-b) > max_radius_diff_)
  {
    throw Error_MotionPlanning_CenterPointDifferentRadius();
  }

  // compute the rotation angle
  double alpha = cosines(a,b,c);

  KDL::RotationalInterpolation* rot_interpo = new KDL::RotationalInterpolation_SingleAxis();
  try
  {
    KDL::Path* circ = new KDL::Path_Circle(start_pose,
                                           center_point,
                                           goal_pose.p,
                                           goal_pose.M,
                                           alpha,
                                           rot_interpo,
                                           eqradius,
                                           true /* take ownership of RotationalInterpolation */);
    return std::unique_ptr<KDL::Path>(circ);
  }
  catch(KDL::Error_MotionPlanning &)
  {
    delete rot_interpo; // in case we could not construct the Path object, avoid a memory leak
    throw; // and pass the exception on to the caller
  }
}

std::unique_ptr<KDL::Path> PathCircleGenerator::circleFromInterim(
    const KDL::Frame &start_pose,
    const KDL::Frame &goal_pose,
    const KDL::Vector &interim_point,
    double eqradius
    )
{
  // compute the center point from interim point
  // triangle edges
  const KDL::Vector t = interim_point - start_pose.p;
  const KDL::Vector u = goal_pose.p - start_pose.p;
  const KDL::Vector v = goal_pose.p - interim_point;
  // triangle normal
  const KDL::Vector w = t*u;

  // circle center
  const KDL::Vector center_point = start_pose.p + (u*dot(t,t)*dot(u,v) - t*dot(u,u)*dot(t,v))* 0.5/pow(w.Norm(),2);

  // compute the rotation angle
  double interim_angle = cosines(t.Norm(), v.Norm(), u.Norm());
  double a = (start_pose.p - center_point).Norm();
  double b = (goal_pose.p - center_point).Norm();
  double c = (start_pose.p - goal_pose.p).Norm();
  // compute the rotation angle
  double alpha = cosines(a,b,c);
  // rotation angle is an acute angle
  //rotation angle is an obtuse angle
  if(interim_angle < M_PI/2)
  {
    alpha = 2*M_PI - alpha;
  }

  KDL::RotationalInterpolation* rot_interpo = new KDL::RotationalInterpolation_SingleAxis();
  try
  {
    KDL::Path_Circle* circ = new KDL::Path_Circle(start_pose,
                                                  center_point,
                                                  interim_point,
                                                  goal_pose.M,
                                                  alpha,
                                                  rot_interpo,
                                                  eqradius,
                                                  true /* take ownership of RotationalInterpolation */);
    return std::unique_ptr<KDL::Path>(circ);
  }
  catch(KDL::Error_MotionPlanning &)
  {
    delete rot_interpo; // in case we could not construct the Path object, avoid a memory leak
    throw; // and pass the exception on to the caller
  }
}

double PathCircleGenerator::cosines(const double a, const double b, const double c)
{
   return acos(std::max(std::min((pow(a,2) + pow(b,2) - pow(c,2))/(2.0*a*b), 1.0), -1.0));
}

}
