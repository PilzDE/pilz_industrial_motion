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

#ifndef XML_TESTDATA_LOADER_H
#define XML_TESTDATA_LOADER_H

#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <moveit/robot_model/robot_model.h>

#include "pilz_industrial_motion_testutils/testdata_loader.h"

namespace pt = boost::property_tree;
namespace pilz_industrial_motion_testutils
{

/**
 * @brief Implements a test data loader which uses a xml file
 * to store the test data.
 *
 * The Xml-file has the following structure:
 *
 * <testdata>
 *
 *  <poses>
 *    <pos name="MyTestPos1">
 *      <group name="manipulator">
 *        <joints>j1 j2 j3 j4 j5 j6</joints>
 *        <xyzQuat link_name="prbt_tcp">x y z wx wy wz w</xyzQuat>
 *      </group>
 *      <group name="gripper">
 *        <joints>j_gripper</joints>
 *      </group>
 *    </pos>
 *
 *    <pos name="MyTestPos2">
 *      <group name="manipulator">
 *        <joints>j1 j2 j3 j4 j5 j6</joints>
 *        <xyzQuat link_name="prbt_tcp">x y z wx wy wz w</xyzQuat>
 *      </group>
 *      <group name="gripper">
 *        <joints>j_gripper</joints>
 *      </group>
 *    </pos>
 *  </poses>
 *
 *  <ptps>
 *    <ptp name="MyPtp1">
 *      <startPos>MyTestPos1</startPos>
 *      <endPos>MyTestPos2</endPos>
 *      <vel>0.1</vel>
 *      <acc>0.2</acc>
 *    </ptp>
 *  </ptps>
 *
 *  <lins>
 *    <lin name="MyTestLin1">
 *      <planningGroup>manipulator</planningGroup>
 *      <targetLink>prbt_tcp</targetLink>
 *      <startPos>MyTestPos1</startPos>
 *      <endPos>MyTestPos2</endPos>
 *      <vel>0.3</vel>
 *      <acc>0.4</acc>
 *    </lin>
 *  </lins>
 *
 *  <circs>
 *    <circ name="MyTestCirc1">
 *       <planningGroup>manipulator</planningGroup>
 *       <targetLink>prbt_tcp</targetLink>
 *      <startPos>MyTestPos1</startPos>
 *      <intermediatePos>MyTestPos1</intermediatePos>
 *      <centerPos>MyTestPos2</centerPos>
 *      <endPos>MyTestPos1</endPos>
 *      <vel>0.2</vel>
 *      <acc>0.5</acc>
 *    </circ>
 *  </circs>
 *
 *  <blends>
 *    <blend name="TestBlend">
 *      <blendCmd name="TestPtp" type="ptp" blend_radius="0.2" />
 *      <blendCmd name="MyTestLin1" type="lin" blend_radius="0.01" />
 *      <blendCmd name="MyTestCirc1" type="circ" blend_radius="0" />
 *    </blend>
 *  <blends>
 *
 * </testdata>
 */

class XmlTestdataLoader : public TestdataLoader
{
public:
  XmlTestdataLoader(const std::string &path_filename);
  ~XmlTestdataLoader();

public:
  virtual bool getJoints(const std::string &pos_name, const std::string &group_name,
                                std::vector<double> &dVec) const override;

  virtual bool getPose(const std::string &pose_name, const std::string &group_name,
                       std::vector<double> &dVec) const override;

  virtual bool getLin(const std::string& cmd_name, STestMotionCommand& cmd) const override;

  virtual bool getCirc(const std::string& cmd_name, STestMotionCommand& cmd) const override;

  virtual bool getBlend(const std::string &cmd_name,
                        std::vector<SBlendCmd> &blend_cmds) const override;

private:
  /**
   * @brief Use this function to search for a node (like an pos or cmd) with a given name.
   *
   * @param tree Tree containing the node.
   * @param name Name of node to look for.
   *
   * @param ok States if the node with the given name was found or not.
   * @return The node which we are looking for, or empty node if node with
   * given name was not found.
   */
  const pt::ptree::value_type &findNodeWithName(const boost::property_tree::ptree &tree,
                                                const std::string &name,
                                                bool &ok) const;

  /**
   * @brief Use this function to search for a cmd-node with a given name.
   *
   * @param cmd_name Name of the cmd.
   * @param cmd_type Type of the cmd (ptp, lin, circ, etc.)
   *
   * @param ok States if the cmd-node with the given name was found or not.
   * @return The cmd-node which we are looking for, or empty node if node with
   * given name was not found.
   */
  const pt::ptree::value_type &findCmd(const std::string &cmd_name,
                                       const std::string &cmd_type, bool &ok) const;

  bool getCmd(const std::string &path2cmd, const std::string &cmd_name,
              std::string &group_name, std::string &target_link,
              std::string& start_pos_name, std::string& end_pos_name,
              double &vel, double &acc) const;

private:
  //! @brief Converts string vector to double vector.
  inline static void strVec2doubleVec(std::vector<std::string> &strVec, std::vector<double> &dVec);

private:
  std::string path_filename_;
  pt::ptree tree_ {};

private:
  const std::string empty_str_ {};
  const pt::ptree::value_type empty_value_type_ {};
  const pt::ptree empty_tree_ {};

  const std::string XML_ATTR_STR {"<xmlattr>"};
  const std::string JOINT_STR {"joints"};
  const std::string XYZ_QUAT_STR {"xyzQuat"};
  const std::string XYZ_EULER_STR {"xyzEuler"};

  const std::string PLANNING_GROUP_STR {"planningGroup"};
  const std::string TARGET_LINK_STR {"targetLink"};
  const std::string START_POS_STR {"startPos"};
  const std::string END_POS_STR {"endPos"};
  const std::string INTERMEDIATE_POS_STR {"intermediatePos"};
  const std::string CENTER_POS_STR {"centerPos"};
  const std::string VEL_STR {"vel"};
  const std::string ACC_STR {"acc"};


  const std::string POSES_PATH_STR {"testdata.poses"};
  const std::string PTPS_PATH_STR {"testdata.ptps"};
  const std::string LINS_PATH_STR {"testdata.lins"};
  const std::string CIRCS_PATH_STR {"testdata.circs"};
  const std::string BLENDS_PATH_STR {"testdata.blends"};

  const std::string NAME_PATH_STR {XML_ATTR_STR + ".name"};
  const std::string CMD_TYPE_PATH_STR {XML_ATTR_STR + ".type"};
  const std::string BLEND_RADIUS_PATH_STR {XML_ATTR_STR + ".blend_radius"};
  const std::string LINK_NAME_PATH_STR {XML_ATTR_STR + ".linkName"};

};

void XmlTestdataLoader::strVec2doubleVec(std::vector<std::string> &strVec, std::vector<double> &dVec)
{
  dVec.resize(strVec.size());
  std::transform(strVec.begin(), strVec.end(), dVec.begin(), [](const std::string& val)
  {
    return std::stod(val);
  });
}
}

#endif // XML_TESTDATA_LOADER_H
