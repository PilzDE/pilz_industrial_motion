#!/usr/bin/env python
# Copyright (c) 2018 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from geometry_msgs.msg import Point
from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *
import math
import rospy

__REQUIRED_API_VERSION__ = "1"

def start_program():
    print("Executing " + __file__)

    r = Robot(__REQUIRED_API_VERSION__)

    L = 0.2
    M = 0.1

    P1 = Pose(position=Point(0.2, 0, 0.8))
    P2 = Pose(position=Point(P1.position.x+L, P1.position.y+L, P1.position.z-M))
    P3 = Pose(position=Point(P1.position.x, P1.position.y+2*L, P1.position.z-2*M))
    P4 = Pose(position=Point(P1.position.x, P1.position.y+L, P1.position.z-M))

    ptp1 = Ptp(goal=P1, acc_scale=0.6)
    ptp2 = Ptp(goal=P2, acc_scale=0.6)
    ptp3 = Ptp(goal=P3, acc_scale=0.6)
    ptp4 = Ptp(goal=P4, acc_scale=0.6)
    lin1 = Lin(goal=P1, acc_scale=0.6)
    lin2 = Lin(goal=P2, acc_scale=0.6)
    lin3 = Lin(goal=P3, acc_scale=0.6)
    lin4 = Lin(goal=P4, acc_scale=0.6)

    circ3_interim_2 = Circ(goal=P3, interim=P2.position, acc_scale=0.4)
    circ1_center_2 = Circ(goal=P1, center=P2.position, acc_scale=0.4)

    r.move(ptp1) #PTP_12
    r.move(ptp2) #PTP_23
    r.move(ptp3) #PTP_34
    r.move(ptp4) #PTP_41
    r.move(ptp1)

    r.move(lin1) #LIN_12
    r.move(lin2) #LIN_23
    r.move(lin3) #LIN_34
    r.move(lin4) #LIN_41
    r.move(lin1)

    circ3_interim_2 = Circ(goal=P3, interim=P2.position, acc_scale=0.4)
    circ1_center_2 = Circ(goal=P1, center=P2.position, acc_scale=0.4)

    for radius in [0, 0.1]:
        r.move(Ptp(goal=[0,0,0,0,0,0]))

        seq = Sequence()
        seq.append(ptp1, blend_radius=radius)
        seq.append(circ3_interim_2, blend_radius=radius)
        seq.append(ptp2, blend_radius=radius)
        seq.append(lin3, blend_radius=radius)
        seq.append(circ1_center_2, blend_radius=radius)
        seq.append(lin2, blend_radius=radius)
        seq.append(ptp3)

        r.move(seq)




if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('robot_program_node')

    start_program()

