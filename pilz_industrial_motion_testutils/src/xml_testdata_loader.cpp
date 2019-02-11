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

#include "pilz_industrial_motion_testutils/exception_types.h"

namespace pt = boost::property_tree;
namespace pilz_industrial_motion_testutils
{

template<class CmdType>
class CmdGetterAdapter : public XmlTestdataLoader::AbstractCmdGetterAdapter
{
public:
  using FuncType = std::function<CmdType(const std::string&)>;

  CmdGetterAdapter(FuncType func)
    : AbstractCmdGetterAdapter()
    , func_(func)
  {
  }

public:
  MotionCmdUPtr getCmd(const std::string& cmd_name) const override
  {
    // We cannot create a derived class object and then copy
    // it into the memory space of a base class object because this
    // will cause "slicing". Therefore, we create a dervied class object
    // first and then let a base class pointer point to the memory
    // space of the derived class.
    std::unique_ptr<CmdType> cmd_ptr { new CmdType() };
    CmdType& cmd {*cmd_ptr};
    cmd = func_(cmd_name);
    return MotionCmdUPtr(std::move(cmd_ptr));
  }

private:
  FuncType func_;
};

XmlTestdataLoader::XmlTestdataLoader(const std::string &path_filename)
  : TestdataLoader()
  , path_filename_(path_filename)
{
  // Parse the XML into the property tree.
  pt::read_xml(path_filename_, tree_, pt::xml_parser::no_comments);

  using std::placeholders::_1;
  cmd_getter_funcs_["ptp"] = AbstractCmdGetterUPtr(new CmdGetterAdapter<PtpJoint>(std::bind(&XmlTestdataLoader::getPtpJoint, this, _1)));
  cmd_getter_funcs_["ptp_joint_cart"] = AbstractCmdGetterUPtr(new CmdGetterAdapter<PtpJointCart>(std::bind(&XmlTestdataLoader::getPtpJointCart, this, _1)));
  cmd_getter_funcs_["ptp_cart_cart"] = AbstractCmdGetterUPtr(new CmdGetterAdapter<PtpCart>(std::bind(&XmlTestdataLoader::getPtpCart, this, _1)));

  cmd_getter_funcs_["lin"] = AbstractCmdGetterUPtr(new CmdGetterAdapter<LinJoint>(std::bind(&XmlTestdataLoader::getLinJoint, this, _1)));
  cmd_getter_funcs_["lin_cart"] = AbstractCmdGetterUPtr(new CmdGetterAdapter<LinCart>(std::bind(&XmlTestdataLoader::getLinCart, this, _1)));

  cmd_getter_funcs_["circ_center_cart"] = AbstractCmdGetterUPtr(new CmdGetterAdapter<CircCenterCart>(std::bind(&XmlTestdataLoader::getCircCartCenterCart, this, _1)));
  cmd_getter_funcs_["circ_interim_cart"] = AbstractCmdGetterUPtr(new CmdGetterAdapter<CircInterimCart>(std::bind(&XmlTestdataLoader::getCircCartInterimCart, this, _1)));
}

XmlTestdataLoader::XmlTestdataLoader(const std::string &path_filename,
                                     moveit::core::RobotModelConstPtr robot_model)
  : XmlTestdataLoader(path_filename)
{
  setRobotModel(robot_model);
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

const pt::ptree::value_type& XmlTestdataLoader::findNodeWithName(const boost::property_tree::ptree &tree,
                                                                 const std::string &name) const
{
  // Search for node with given name.
  for (const pt::ptree::value_type& val : tree)
  {
    // Ignore attributes which are always the first element of a tree.
    if (val.first == XML_ATTR_STR){ continue; }

    const auto& node = val.second.get_child(NAME_PATH_STR, empty_tree_);
    if (node == empty_tree_) { break; }
    if (node.data() == name) { return val; }
  }

  std::string msg;
  msg.append("Node \"").append(name).append("\" not found.");
  throw TestDataLoaderReadingException(msg);
}

bool XmlTestdataLoader::getJoints(const std::string &pos_name, const std::string &group_name,
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
  const pt::ptree::value_type& pose_node { findNodeWithName(poses_tree, pos_name, ok) };
  if (!ok)
  {
    ROS_ERROR_STREAM("Pos '" << pos_name << "' not found.");
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

JointConfiguration XmlTestdataLoader::getJoints(const std::string &pos_name,
                                                const std::string &group_name) const
{
  // Search for node with given name.
  const auto& poses_tree {tree_.get_child(POSES_PATH_STR, empty_tree_)};
  if (poses_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No poses found.");
  }

  const auto& pose_node {findNodeWithName(poses_tree, pos_name)};

  // Search group node with given name.
  const auto& group_tree {pose_node.second};
  if (group_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No groups found.");
  }

  const auto& group_node {findNodeWithName(group_tree, group_name)};

  // Read joint values
  const auto& joint_tree {group_node.second.get_child(JOINT_STR, empty_tree_)};
  if (joint_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No joint node found.");
  }

  std::vector<std::string> strs;
  boost::split(strs,  joint_tree.data(), boost::is_any_of(" "));
  return JointConfiguration(group_name, strVec2doubleVec(strs), robot_model_);
}

bool XmlTestdataLoader::getPose(const std::string &pos_name, const std::string &group_name,
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
  const pt::ptree::value_type& pose_node { findNodeWithName(poses_tree, pos_name, ok) };
  if (!ok)
  {
    ROS_ERROR_STREAM("Pos '" << pos_name << "' not found.");
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

CartesianConfiguration XmlTestdataLoader::getPose(const std::string &pos_name,
                                                  const std::string &group_name) const
{
  // Search for node with given name.
  const auto& poses_tree = tree_.get_child(POSES_PATH_STR, empty_tree_);
  if (poses_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No poses found.");
  }

  const auto& pose_node {findNodeWithName(poses_tree, pos_name)};

  // Search group node with given name.
  const auto& group_tree = pose_node.second;
  if (group_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No groups found.");
  }

  const auto& group_node {findNodeWithName(group_tree, group_name)};

  // Read joint values
  const auto& xyzQuat_tree {group_node.second.get_child(XYZ_QUAT_STR, empty_tree_)};
  if (xyzQuat_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No cartesian node found.");
  }

  const boost::property_tree::ptree& link_name_attr = xyzQuat_tree.get_child(LINK_NAME_PATH_STR, empty_tree_);
  if (link_name_attr == empty_tree_)
  {
    throw TestDataLoaderReadingException("No link name found.");
  }
  std::string link_name {link_name_attr.data()};

  std::vector<std::string> strs;
  boost::split(strs,  xyzQuat_tree.data(), boost::is_any_of(" "));
  return CartesianConfiguration(group_name, link_name,
                                strVec2doubleVec(strs), robot_model_);
}

PtpJoint XmlTestdataLoader::getPtpJoint(const std::string& cmd_name) const
{
  std::string start_pos_name, goal_pos_name, planning_group, target_link;
  double vel_scale, acc_scale;
  if(!getCmd(PTPS_PATH_STR, cmd_name, planning_group, target_link,
             start_pos_name, goal_pos_name, vel_scale, acc_scale))
  {
    throw TestDataLoaderReadingException("Did not find \"" + cmd_name +  "\"");
  }

  PtpJoint cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setTargetLink(target_link);
  cmd.setVelocityScale(vel_scale);
  cmd.setAccelerationScale(acc_scale);

  cmd.setStartConfiguration(getJoints(start_pos_name, planning_group));
  cmd.setGoalConfiguration(getJoints(goal_pos_name, planning_group));

  return cmd;
}

PtpJointCart XmlTestdataLoader::getPtpJointCart(const std::string& cmd_name) const
{
  std::string start_pos_name, goal_pos_name, planning_group, target_link;
  double vel_scale, acc_scale;
  if(!getCmd(PTPS_PATH_STR, cmd_name, planning_group, target_link,
             start_pos_name, goal_pos_name, vel_scale, acc_scale))
  {
    throw TestDataLoaderReadingException("Did not find \"" + cmd_name +  "\"");
  }

  PtpJointCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setTargetLink(target_link);
  cmd.setVelocityScale(vel_scale);
  cmd.setAccelerationScale(acc_scale);

  cmd.setStartConfiguration(getJoints(start_pos_name, planning_group));
  cmd.setGoalConfiguration(getPose(goal_pos_name, planning_group));

  return cmd;
}

PtpCart XmlTestdataLoader::getPtpCart(const std::string& cmd_name) const
{
  std::string start_pos_name, goal_pos_name, planning_group, target_link;
  double vel_scale, acc_scale;
  if(!getCmd(PTPS_PATH_STR, cmd_name, planning_group, target_link,
             start_pos_name, goal_pos_name, vel_scale, acc_scale))
  {
    throw TestDataLoaderReadingException("Did not find \"" + cmd_name +  "\"");
  }

  PtpCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setTargetLink(target_link);
  cmd.setVelocityScale(vel_scale);
  cmd.setAccelerationScale(acc_scale);

  cmd.setStartConfiguration(getPose(start_pos_name, planning_group));
  cmd.setGoalConfiguration(getPose(goal_pos_name, planning_group));

  return cmd;
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

LinJoint XmlTestdataLoader::getLinJoint(const std::string& cmd_name) const
{
  std::string start_pos_name, goal_pos_name, planning_group, target_link;
  double vel_scale, acc_scale;
  if(!getCmd(LINS_PATH_STR, cmd_name, planning_group, target_link,
             start_pos_name, goal_pos_name, vel_scale, acc_scale))
  {
    throw TestDataLoaderReadingException("Could not load \"" + cmd_name +  "\"");
  }

  LinJoint cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setTargetLink(target_link);
  cmd.setVelocityScale(vel_scale);
  cmd.setAccelerationScale(acc_scale);

  cmd.setStartConfiguration(getJoints(start_pos_name, planning_group));
  cmd.setGoalConfiguration(getJoints(goal_pos_name, planning_group));

  return cmd;
}

LinCart XmlTestdataLoader::getLinCart(const std::string& cmd_name) const
{
  std::string start_pos_name, goal_pos_name, planning_group, target_link;
  double vel_scale, acc_scale;
  if(!getCmd(LINS_PATH_STR, cmd_name, planning_group, target_link,
             start_pos_name, goal_pos_name, vel_scale, acc_scale))
  {
    throw TestDataLoaderReadingException("Could not load \"" + cmd_name +  "\"");
  }

  LinCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setTargetLink(target_link);
  cmd.setVelocityScale(vel_scale);
  cmd.setAccelerationScale(acc_scale);

  cmd.setStartConfiguration(getPose(start_pos_name, planning_group));
  cmd.setGoalConfiguration(getPose(goal_pos_name, planning_group));

  return cmd;
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

const pt::ptree::value_type & XmlTestdataLoader::findCmd(const std::string &cmd_name,
                                                         const std::string &cmd_type) const
{
  // Search for node with given name.
  const boost::property_tree::ptree& cmds_tree = tree_.get_child(cmd_type, empty_tree_);
  if (cmds_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No cmd of type \"" + cmd_type +  "\" found");
  }

  return findNodeWithName(cmds_tree, cmd_name);
}

bool XmlTestdataLoader::getCmd(const std::string &path2cmd,
                               const std::string &cmd_name,
                               std::string &group_name,
                               std::string &target_link,
                               std::string& start_pos_name,
                               std::string& end_pos_name,
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

  start_pos_name = cmd_node.second.get<std::string>(START_POS_STR, empty_str_);
  if (start_pos_name.empty())
  {
    ROS_ERROR("No start pos found.");
    return false;
  }

  end_pos_name = cmd_node.second.get<std::string>(END_POS_STR, empty_str_);
  if (end_pos_name.empty())
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

CartesianCenter XmlTestdataLoader::getCartesianCenter(const std::string &cmd_name,
                                                      const std::string &planning_group) const
{
  const pt::ptree::value_type &cmd_node { findCmd(cmd_name, CIRCS_PATH_STR) };
  std::string aux_pos_name;
  try
  {
    aux_pos_name = cmd_node.second.get<std::string>(CENTER_POS_STR);
  }
  catch(...)
  {
    throw TestDataLoaderReadingException("Did not find center of circ");
  }

  CartesianCenter aux;
  aux.setConfiguration(getPose(aux_pos_name, planning_group));
  return aux;
}

CartesianInterim XmlTestdataLoader::getCartesianInterim(const std::string &cmd_name,
                                                        const std::string &planning_group) const
{
  const pt::ptree::value_type &cmd_node { findCmd(cmd_name, CIRCS_PATH_STR) };
  std::string aux_pos_name;
  try
  {
    aux_pos_name = cmd_node.second.get<std::string>(INTERMEDIATE_POS_STR);
  }
  catch(...)
  {
    throw TestDataLoaderReadingException("Did not find interim of circ");
  }

  CartesianInterim aux;
  aux.setConfiguration(getPose(aux_pos_name, planning_group));
  return aux;
}

CircCenterCart XmlTestdataLoader::getCircCartCenterCart(const std::string &cmd_name) const
{
  std::string start_pos_name, goal_pos_name, planning_group, target_link;
  double vel_scale, acc_scale;
  if(!getCmd(CIRCS_PATH_STR, cmd_name, planning_group, target_link,
             start_pos_name, goal_pos_name, vel_scale, acc_scale))
  {
    throw TestDataLoaderReadingException("Did not \"" + cmd_name +  "\"");
  }

  CircCenterCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setTargetLink(target_link);
  cmd.setVelocityScale(vel_scale);
  cmd.setAccelerationScale(acc_scale);

  cmd.setStartConfiguration(getPose(start_pos_name, planning_group));
  cmd.setAuxiliaryConfiguration(getCartesianCenter(cmd_name, planning_group));
  cmd.setGoalConfiguration(getPose(goal_pos_name, planning_group));

  return cmd;
}

CircInterimCart XmlTestdataLoader::getCircCartInterimCart(const std::string &cmd_name) const
{
  std::string start_pos_name, goal_pos_name, planning_group, target_link;
  double vel_scale, acc_scale;
  if(!getCmd(CIRCS_PATH_STR, cmd_name, planning_group, target_link,
             start_pos_name, goal_pos_name, vel_scale, acc_scale))
  {
    throw TestDataLoaderReadingException("Did not \"" + cmd_name +  "\"");
  }

  CircInterimCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setTargetLink(target_link);
  cmd.setVelocityScale(vel_scale);
  cmd.setAccelerationScale(acc_scale);

  cmd.setStartConfiguration(getPose(start_pos_name, planning_group));
  cmd.setAuxiliaryConfiguration(getCartesianInterim(cmd_name, planning_group));
  cmd.setGoalConfiguration(getPose(goal_pos_name, planning_group));

  return cmd;
}

CircJointCenterCart XmlTestdataLoader::getCircJointCenterCart(const std::string &cmd_name) const
{
  std::string start_pos_name, goal_pos_name, planning_group, target_link;
  double vel_scale, acc_scale;
  if(!getCmd(CIRCS_PATH_STR, cmd_name, planning_group, target_link,
             start_pos_name, goal_pos_name, vel_scale, acc_scale))
  {
    throw TestDataLoaderReadingException("Did not \"" + cmd_name +  "\"");
  }

  CircJointCenterCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setTargetLink(target_link);
  cmd.setVelocityScale(vel_scale);
  cmd.setAccelerationScale(acc_scale);

  cmd.setStartConfiguration(getJoints(start_pos_name, planning_group));
  cmd.setAuxiliaryConfiguration(getCartesianCenter(cmd_name, planning_group));
  cmd.setGoalConfiguration(getJoints(goal_pos_name, planning_group));

  return cmd;
}

Sequence XmlTestdataLoader::getSequence(const std::string &cmd_name) const
{
  Sequence seq;

  // Find sequence with given name and loop over all its cmds
  const auto& sequence_cmd_tree {findCmd(cmd_name, SEQUENCE_PATH_STR).second};
  for (const pt::ptree::value_type& seq_cmd : sequence_cmd_tree)
  {
    // Ignore attributes which are always the first element of a tree.
    if (seq_cmd.first == XML_ATTR_STR){ continue; }

    // Get name of blend cmd.
    const boost::property_tree::ptree& cmd_name_attr = seq_cmd.second.get_child(NAME_PATH_STR, empty_tree_);
    if (cmd_name_attr == empty_tree_)
    {
      throw TestDataLoaderReadingException("Did not find name of sequence cmd");
    }

    std::string cmd_name {cmd_name_attr.data()};

    // Get type of blend cmd
    const boost::property_tree::ptree& type_name_attr {seq_cmd.second.get_child(CMD_TYPE_PATH_STR, empty_tree_)};
    if (type_name_attr == empty_tree_)
    {
      throw TestDataLoaderReadingException("Did not find type of sequence cmd \"" + cmd_name +  "\"");
    }
    std::string cmd_type {type_name_attr.data()};

    // Get blend radius of blend cmd.
    double blend_radius {seq_cmd.second.get<double>(BLEND_RADIUS_PATH_STR, DEFAULT_BLEND_RADIUS)};

    // Read current command from test data
    MotionCmdUPtr curr_cmd {std::move(cmd_getter_funcs_.at(cmd_type)->getCmd(cmd_name))};

    // Add command to sequence
    seq.add(std::move(curr_cmd), blend_radius);
  }

  return seq;
}


}
