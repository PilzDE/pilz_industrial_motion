/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pilz_industrial_motion_testutils/xml_testdata_loader.h"

#include <iostream>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>

#include "pilz_industrial_motion_testutils/default_values.h"

namespace pt = boost::property_tree;
namespace pilz_industrial_motion_testutils
{

XmlTestdataLoader::XmlTestdataLoader(const std::string &path_filename)
  : TestdataLoader()
  , path_filename_(path_filename)
{
  // Parse the XML into the property tree.
  pt::read_xml(path_filename_, tree_, pt::xml_parser::no_comments);
}

XmlTestdataLoader::~XmlTestdataLoader()
{

}

const pt::ptree::value_type& XmlTestdataLoader::findNodeWithName(const boost::property_tree::ptree& tree,
                                                                 const std::string &name,
                                                                 bool &ok) const
{
  // Search for node with given name.
  for (const pt::ptree::value_type& val : tree)
  {
    // Ignore attributes which are always the first element of a tree.
    if (val.first == XML_ATTR_STR){ continue; }

    const boost::property_tree::ptree& node = val.second.get_child(NAME_PATH_STR, empty_tree_);
    if (node == empty_tree_) { ok = false; return empty_value_type_; }
    if (node.data() == name)
    {
      ok = true;
      return val;
    }
  }
  // In case the node is not found.
  ok = false;
  return empty_value_type_;
}

bool XmlTestdataLoader::getJoints(const std::string &pose_name, const std::string &group_name,
                                  std::vector<double> &dVec) const
{
  // Search for node with given name.
  bool ok {false};
  const boost::property_tree::ptree& poses_tree = tree_.get_child(POSES_PATH_STR, empty_tree_);
  if (poses_tree == empty_tree_)
  {
    ROS_ERROR("No poses found.");
    return false;
  }
  const pt::ptree::value_type& pose_node { findNodeWithName(poses_tree, pose_name, ok) };
  if (!ok)
  {
    ROS_ERROR_STREAM("Pos '" << pose_name << "' not found.");
    return false;
  }

  // Search group node with given name.
  const auto& group_tree = pose_node.second;
  if (group_tree == empty_tree_)
  {
    ROS_ERROR("No groups found.");
    return false;
  }
  ok = false;
  const pt::ptree::value_type& group_node { findNodeWithName(group_tree, group_name, ok) };
  if (!ok)
  {
    ROS_ERROR_STREAM("Group '" << group_name << "' not found.");
    return false;
  }

  // Read joint values
  const boost::property_tree::ptree& joint_tree = group_node.second.get_child(JOINT_STR, empty_tree_);
  if (joint_tree == empty_tree_)
  {
    ROS_ERROR("No joint node found.");
    return false;
  }
  std::vector<std::string> strs;
  boost::split(strs,  joint_tree.data(), boost::is_any_of(" "));
  strVec2doubleVec(strs, dVec);
  return true;
}

bool XmlTestdataLoader::getPose(const std::string &pose_name, const std::string &group_name,
                                std::vector<double> &dVec) const
{
  // Search for node with given name.
  bool ok {false};
  const boost::property_tree::ptree& poses_tree = tree_.get_child(POSES_PATH_STR, empty_tree_);
  if (poses_tree == empty_tree_)
  {
    ROS_ERROR("No poses found.");
    return false;
  }
  const pt::ptree::value_type& pose_node { findNodeWithName(poses_tree, pose_name, ok) };
  if (!ok)
  {
    ROS_ERROR_STREAM("Pos '" << pose_name << "' not found.");
    return false;
  }

  // Search group node with given name.
  const auto& group_tree = pose_node.second;
  if (group_tree == empty_tree_)
  {
    ROS_ERROR("No groups found.");
    return false;
  }
  ok = false;
  const pt::ptree::value_type& group_node { findNodeWithName(group_tree, group_name, ok) };
  if (!ok)
  {
    ROS_ERROR_STREAM("Group '" << group_name << "' not found.");
    return false;
  }

  // Read joint values
  const boost::property_tree::ptree& xyzQuat_tree = group_node.second.get_child(XYZ_QUAT_STR, empty_tree_);
  if (xyzQuat_tree == empty_tree_)
  {
    ROS_ERROR("No cartesian node found.");
    return false;
  }
  std::vector<std::string> strs;
  boost::split(strs,  xyzQuat_tree.data(), boost::is_any_of(" "));
  strVec2doubleVec(strs, dVec);

//  const boost::property_tree::ptree& linkNode = xyzEuler_tree.get_child(LINK_NAME_PATH_STR, empty_tree_);
//  if (linkNode == empty_tree_) { return false; }
//  link_name = linkNode.data();

  return true;
}

bool XmlTestdataLoader::getLin(const std::string &cmd_name, STestMotionCommand &cmd) const
{
  std::string start_pos_name, goal_pos_name;
  if(getCmd(LINS_PATH_STR, cmd_name, cmd.planning_group, cmd.target_link,
            start_pos_name, goal_pos_name, cmd.vel_scale, cmd.acc_scale))
  {
    if(getJoints(start_pos_name, cmd.planning_group, cmd.start_position) &&
       getJoints(goal_pos_name, cmd.planning_group, cmd.goal_position))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

const pt::ptree::value_type& XmlTestdataLoader::findCmd(const std::string &cmd_name,
                                                        const std::string &cmd_type,
                                                        bool &ok) const
{
  // Search for node with given name.
  const boost::property_tree::ptree& cmds_tree = tree_.get_child(cmd_type, empty_tree_);
  if (cmds_tree == empty_tree_)
  {
    ROS_ERROR_STREAM("No cmd of type '" << cmd_type << "' found.");
    ok = false;
    return empty_value_type_;
  }
  const pt::ptree::value_type& cmd_node { findNodeWithName(cmds_tree, cmd_name, ok) };
  if (!ok)
  {
    ROS_ERROR_STREAM("Cmd '" << cmd_name << "' not found.");
    return empty_value_type_;
  }
  return cmd_node;
}

bool XmlTestdataLoader::getCmd(const std::string &path2cmd,
                               const std::string &cmd_name,
                               std::string &group_name,
                               std::string &target_link,
                               std::string& start_pose_name,
                               std::string& end_pose_name,
                               double &vel,
                               double &acc) const
{
  bool ok {false};
  const pt::ptree::value_type &cmd_node { findCmd(cmd_name, path2cmd, ok) };
  if (!ok){ return false; }

  group_name = cmd_node.second.get<std::string>(PLANNING_GROUP_STR, empty_str_);
  if (group_name.empty())
  {
    ROS_ERROR("No planning group name found.");
    return false;
  }

  target_link = cmd_node.second.get<std::string>(TARGET_LINK_STR, empty_str_);
  if (target_link.empty())
  {
    ROS_ERROR("No target link name found.");
    return false;
  }

  start_pose_name = cmd_node.second.get<std::string>(START_POS_STR, empty_str_);
  if (start_pose_name.empty())
  {
    ROS_ERROR("No start pos found.");
    return false;
  }

  end_pose_name = cmd_node.second.get<std::string>(END_POS_STR, empty_str_);
  if (end_pose_name.empty())
  {
    ROS_ERROR("No end pos found.");
    return false;
  }

  // Optional parameters
  vel = cmd_node.second.get<double>(VEL_STR, DEFAULT_VEL);
  acc = cmd_node.second.get<double>(ACC_STR, DEFAULT_ACC);
  return true;
}

bool XmlTestdataLoader::getCirc(const std::string &cmd_name, STestMotionCommand &cmd) const
{
  // get start and goal
  std::string start_pos_name, goal_pos_name;
  if(!getCmd(CIRCS_PATH_STR, cmd_name, cmd.planning_group, cmd.target_link,
            start_pos_name, goal_pos_name, cmd.vel_scale, cmd.acc_scale))
  {
    ROS_ERROR_STREAM(cmd_name << " does not exist.");
    return false;
  }

  if(!getJoints(start_pos_name, cmd.planning_group, cmd.start_position) ||
     !getPose(start_pos_name, cmd.planning_group, cmd.start_pose))
  {
    ROS_ERROR_STREAM("Joint position and Cartesian pose must be given for start state.");
    return false;
  }

  if(!getJoints(goal_pos_name, cmd.planning_group, cmd.goal_position) ||
     !getPose(goal_pos_name, cmd.planning_group, cmd.goal_pose))
  {
    ROS_ERROR_STREAM("Joint position and Cartesian pose must be given for goal state.");
    return false;
  }

  // Look for auxiliary point
  bool ok {false};
  const pt::ptree::value_type &cmd_node { findCmd(cmd_name, CIRCS_PATH_STR, ok) };
  if (!ok){ return false; }

  std::string aux_pos_str;
  switch(cmd.aux_pos_type)
  {
  case ECircAuxPosType::eCENTER:
    aux_pos_str = CENTER_POS_STR;
    break;
  case ECircAuxPosType::eINTERMEDIATE:
    aux_pos_str = INTERMEDIATE_POS_STR;
    break;
  }

  std::string aux_pos_name;
  try
  {
    aux_pos_name = cmd_node.second.get<std::string>(aux_pos_str);
  }
  catch(const pt::ptree_bad_path &e)
  {
    ROS_ERROR("Did not find auxiliary pos. Exception: %s", e.what());
    return false;
  }

  if(!getPose(aux_pos_name, cmd.planning_group, cmd.aux_pose))
  {
    ROS_ERROR("Cartesian pose must be given for auxiliary point.");
    return false;
  }

  return true;
}

bool XmlTestdataLoader::getBlend(const std::string &cmd_name,
                                 std::vector<SBlendCmd> &blend_cmds) const
{
  // Find blend cmd with given name
  bool ok {false};
  const pt::ptree::value_type &blend_node { findCmd(cmd_name, BLENDS_PATH_STR, ok) };
  if (!ok){ return false; }

  // Loop over all blend cmds.
  const auto& blend_cmd_tree = blend_node.second;
  for (const pt::ptree::value_type& blend_cmd : blend_cmd_tree)
  {
    // Ignore attributes which are always the first element of a tree.
    if (blend_cmd.first == XML_ATTR_STR){ continue; }

    // Get name of blend cmd.
    const boost::property_tree::ptree& cmd_name_attr = blend_cmd.second.get_child(NAME_PATH_STR, empty_tree_);
    if (cmd_name_attr == empty_tree_)
    {
      ROS_ERROR("Did not find name of blend cmd.");
      return false;
    }
    SBlendCmd blend_cmd_data;
    blend_cmd_data.cmd_name = cmd_name_attr.data();

    // Get type of blend cmd.
    const boost::property_tree::ptree& type_name_attr = blend_cmd.second.get_child(CMD_TYPE_PATH_STR, empty_tree_);
    if (type_name_attr == empty_tree_)
    {
      ROS_ERROR_STREAM("Did not find type of blend cmd '" << blend_cmd_data.cmd_name << "'." );
      return false;
    }
    blend_cmd_data.cmd_type = type_name_attr.data();

    // Get blend radius of blend cmd.
    blend_cmd_data.blend_radius = blend_cmd.second.get<double>(BLEND_RADIUS_PATH_STR, DEFAULT_BLEND_RADIUS);

    // Add blend cmd to container.
    blend_cmds.push_back(blend_cmd_data);
  }
  return true;
}


}
