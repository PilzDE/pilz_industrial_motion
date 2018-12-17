#
# Copyright (c) 2018 Pilz GmbH & Co. KG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import xml.etree.ElementTree as ET
import re
from geometry_msgs.msg import Pose
from math import pi

DEFAULT_VEL = 0.01
DEFAULT_ACC = 0.01
DEFAULT_BLEND_RADIUS = 0.01

NAME_STR = "name"
TYPE_STR = "type"
BLEND_RADIUS_STR = "blend_radius"

START_POS_STR = "startPos"
END_POS_STR = "endPos"
PLANNING_GROUP_STR = "planningGroup"
LINK_NAME_STR = "targetLink"
INTERIM_POS_STR = "interimPos"
CENTER_POS_STR = "centerPos"
AUXILIARY_POS_STR = "auxiliaryPos"
DEFAULT_LINK_NAME = "prbt_tcp"

VEL_STR = "vel"
ACC_STR = "acc"

_PATH_TO_JOINTS = "./poses/pos[@name='{pose_name}']/group[@name='{group_name}']/joints"
_PATH_TO_POSE = "./poses/pos[@name='{pose_name}']/group[@name='{group_name}']/xyzQuat"
_PATH_TO_PTP = "./ptps/ptp[@name='{name_of_cmd}']"
_PATH_TO_LIN = "./lins/lin[@name='{name_of_cmd}']"
_PATH_TO_CIRC = "./circs/circ[@name='{name_of_cmd}']"
_PATH_TO_SEQUENCE = "./sequences/sequence[@name='{name_of_cmd}']"


class XmlTestdataLoader:

    def __init__(self, path_to_xml_file):
        self._tree = ET.parse(path_to_xml_file)
        self._root = self._tree.getroot()

    # Returns the joint values for the given group and position.
    def get_joints(self, pose_name, group_name):
        joint_node = self._root.find(_PATH_TO_JOINTS.format(pose_name=pose_name, group_name=group_name))
        if joint_node is None:
            return None
        return [eval(elem) for elem in re.split(r'[^\S\n\t]+', joint_node.text)]

    def get_pose(self, pose_name, group_name):
        node = self._root.find(_PATH_TO_POSE.format(pose_name=pose_name, group_name=group_name))
        if node is None:
            return None
        pose_list = [eval(elem) for elem in re.split(r'[^\S\n\t]+', node.text)]
        pose = Pose()
        pose.position.x = pose_list[0]
        pose.position.y = pose_list[1]
        pose.position.z = pose_list[2]
        pose.orientation.x = pose_list[3]
        pose.orientation.y = pose_list[4]
        pose.orientation.z = pose_list[5]
        pose.orientation.w = pose_list[6]
        return pose

    # Returns the start- and end-position, as well as
    # the velocity and acceleration of the ptp command given by its name.
    # In case of an error 'None' is returned.
    def get_ptp(self, name_of_cmd):
        return self._get_cmd(_PATH_TO_PTP, name_of_cmd)

    # Returns the start- and end-position, as well as
    # the velocity and acceleration of the lin command given by its name.
    # In case of an error 'None' is returned.
    def get_lin(self, name_of_cmd):
        return self._get_cmd(_PATH_TO_LIN, name_of_cmd)

    # Returns the start-, end- and auxility-position, as well as
    # the velocity and acceleration of the circ command given by its name.
    #
    # Please note: It is also necessary to state if the auxiliary point
    # of the circ command is stored as intermediate or center point.
    def get_circ(self, name_of_cmd, auxiliaray_pos_type=INTERIM_POS_STR):
        cmdRes = self._get_cmd(_PATH_TO_CIRC, name_of_cmd)
        if cmdRes is None:
            return None

        cmdNode = self._root.find(_PATH_TO_CIRC.format(name_of_cmd=name_of_cmd))
        if cmdNode is None:
            return None
        auxiliaryNode = cmdNode.find("./{}".format(auxiliaray_pos_type))
        if auxiliaryNode is None:
            return None

        return {START_POS_STR: cmdRes[START_POS_STR], auxiliaray_pos_type: auxiliaryNode.text,
                END_POS_STR: cmdRes[END_POS_STR], VEL_STR: cmdRes[VEL_STR], ACC_STR: cmdRes[ACC_STR],
                PLANNING_GROUP_STR: cmdRes[PLANNING_GROUP_STR], LINK_NAME_STR: cmdRes[LINK_NAME_STR]}

    # Returns a list of dictionaries containing the cmds which make-up the
    # sequence cmd. The cmds in the list are in the order of execution.
    # In case of an error 'None' is returned.
    def get_sequence(self, name_of_cmd):
        # Find the sequence command with the given name
        sequenceNode = self._root.find(_PATH_TO_SEQUENCE.format(name_of_cmd=name_of_cmd))
        if sequenceNode is None:
            return None

        # Loop over all blend commands
        sequenceCmds = []
        for sequenceCmdNode in sequenceNode.getchildren():
            cmd_name = sequenceCmdNode.get(NAME_STR)
            if cmd_name is None:
                return None

            cmd_type = sequenceCmdNode.get(TYPE_STR)
            if cmd_type is None:
                return None

            blend_radius = sequenceCmdNode.get(BLEND_RADIUS_STR, DEFAULT_BLEND_RADIUS)
            sequenceCmds.append({NAME_STR: cmd_name, TYPE_STR: cmd_type, BLEND_RADIUS_STR: blend_radius})

        return sequenceCmds

    # Returns the start- and end-position, as well as
    # the velocity and acceleration of the given command type, given by its name.
    # The values are returned as dictionaries.
    # In case of an error 'None' is returned.
    def _get_cmd(self, path_to_cmd_type, name_of_cmd):
        cmd_node = self._root.find(path_to_cmd_type.format(name_of_cmd=name_of_cmd))
        if cmd_node is None:
            return None

        start_pos_node = cmd_node.find("./{}".format(START_POS_STR))
        if start_pos_node is None:
            return None

        end_pos_node = cmd_node.find("./{}".format(END_POS_STR))
        if end_pos_node is None:
            return None

        planning_group_node = cmd_node.find("./{}".format(PLANNING_GROUP_STR))
        if planning_group_node is None:
            return None

        # Optional parameters
        vel_node = cmd_node.find("./{}".format(VEL_STR))
        vel = DEFAULT_VEL if vel_node is None else float(vel_node.text)

        acc_node = cmd_node.find("./{}".format(ACC_STR))
        acc = DEFAULT_ACC if acc_node is None else float(acc_node.text)

        target_link_node = cmd_node.find("./{}".format(LINK_NAME_STR))
        target_link_name = DEFAULT_LINK_NAME if target_link_node is None else target_link_node.text

        return {START_POS_STR: start_pos_node.text, END_POS_STR: end_pos_node.text, VEL_STR: vel, ACC_STR: acc,
                PLANNING_GROUP_STR: planning_group_node.text, LINK_NAME_STR: target_link_name}
