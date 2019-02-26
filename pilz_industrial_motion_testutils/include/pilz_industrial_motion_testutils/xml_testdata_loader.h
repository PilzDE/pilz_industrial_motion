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
#include <functional>
#include <map>
#include <memory>

#include <boost/property_tree/ptree.hpp>

#include <moveit/robot_model/robot_model.h>

#include "pilz_industrial_motion_testutils/testdata_loader.h"
#include "sequence.h"

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
 *      <joints group_name="manipulator">j1 j2 j3 j4 j5 j6</joints>
 *      <xyzQuat group_name="manipulator" link_name="prbt_tcp">
 *        x y z wx wy wz w
 *        <seed><joints group_name="manipulator">s1 s2 s3 s4 s5 s6</joints></seed>
 *      </xyzQuat>
 *      <joints group_name="gripper">j_gripper</joints>
 *    </pos>
 *
 *    <pos name="MyTestPos2">
 *      <joints group_name="manipulator">j1 j2 j3 j4 j5 j6</joints>
 *      <xyzQuat group_name="manipulator" link_name="prbt_tcp">x y z wx wy wz w</xyzQuat>
 *      <joints group_name="gripper">j_gripper</joints>
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
 *  <sequences>
 *    <blend name="TestBlend">
 *      <sequenceCmd name="TestPtp" type="ptp" blend_radius="0.2" />
 *      <sequenceCmd name="MyTestLin1" type="lin" blend_radius="0.01" />
 *      <sequenceCmd name="MyTestCirc1" type="circ" blend_radius="0" />
 *    </blend>
 *  </sequences>
 *
 * </testdata>
 */

class XmlTestdataLoader : public TestdataLoader
{
public:
  XmlTestdataLoader(const std::string &path_filename);
  XmlTestdataLoader(const std::string &path_filename,
                    moveit::core::RobotModelConstPtr robot_model);
  ~XmlTestdataLoader();

public:
  //! @deprecated Use function using higher level abstraction data class instead.
  virtual bool getJoints(const std::string &pos_name, const std::string &group_name,
                         std::vector<double> &dVec) const override;

  virtual JointConfiguration getJoints(const std::string &pos_name,
                                       const std::string &group_name) const override;

  //! @deprecated Use function using higher level abstraction data class instead.
  virtual bool getPose(const std::string &pos_name, const std::string &group_name,
                       std::vector<double> &dVec) const override;

  virtual CartesianConfiguration getPose(const std::string &pos_name,
                                         const std::string &group_name) const override;

  virtual PtpJoint getPtpJoint(const std::string& cmd_name) const override;
  virtual PtpCart getPtpCart(const std::string& cmd_name) const override;
  virtual PtpJointCart getPtpJointCart(const std::string& cmd_name) const override;

  //! @deprecated Use function using higher level abstraction data class instead.
  virtual bool getLin(const std::string& cmd_name, STestMotionCommand& cmd) const override;

  virtual LinJoint getLinJoint(const std::string& cmd_name) const override;
  virtual LinCart getLinCart(const std::string& cmd_name) const override;
  virtual LinJointCart getLinJointCart(const std::string& cmd_name) const override;


  //! @deprecated Use function using higher level abstraction data class instead.
  virtual bool getCirc(const std::string& cmd_name, STestMotionCommand& cmd) const override;

  virtual CircCenterCart getCircCartCenterCart(const std::string &cmd_name) const override;
  virtual CircInterimCart getCircCartInterimCart(const std::string &cmd_name) const override;
  virtual CircJointCenterCart getCircJointCenterCart(const std::string &cmd_name) const override;

  virtual Sequence getSequence(const std::string &cmd_name) const override;

private:
  /**
   * @deprecated Use function using higher level abstraction data class instead.
   *
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

  const pt::ptree::value_type &findNodeWithName(const boost::property_tree::ptree &tree,
                                                const std::string &name,
                                                const std::string &key,
                                                const std::string &path = "") const;

  /**
   * @deprecated Use function using higher level abstraction data class instead.
   *
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

  const pt::ptree::value_type &findCmd(const std::string &cmd_name,
                                       const std::string& cmd_path,
                                       const std::string &cmd_key) const;

  bool getCmd(const std::string &path2cmd, const std::string &cmd_name,
              std::string &group_name, std::string &target_link,
              std::string& start_pos_name, std::string& end_pos_name,
              double &vel, double &acc) const;

  CartesianCenter getCartesianCenter(const std::string &cmd_name,
                                     const std::string &planning_group) const;

  CartesianInterim getCartesianInterim(const std::string &cmd_name,
                                       const std::string &planning_group) const;

private:
  JointConfiguration getJoints(const boost::property_tree::ptree& joints_tree,
                               const std::string &group_name) const;

private:
  //! @deprecated Use function using higher level abstraction data class instead.
  //! @brief Converts string vector to double vector.
  inline static void strVec2doubleVec(std::vector<std::string> &strVec, std::vector<double> &dVec);

  inline static std::vector<double> strVec2doubleVec(std::vector<std::string> &strVec);

private:
  /**
   * @brief Abstract base class providing a GENERIC getter-function signature
   * which can be used to load DIFFERENT command types (like Ptp, Lin, etc.)
   * from the test data file.
   */
  class AbstractCmdGetterAdapter
  {
  public:
    virtual CmdVariant getCmd(const std::string& /*cmd_name*/) const = 0;
  };

private:
  std::string path_filename_;
  pt::ptree tree_ {};

  using AbstractCmdGetterUPtr = std::unique_ptr<AbstractCmdGetterAdapter>;

  //! Stores the mapping between command type and the getter function
  //! which have to be called.
  //!
  //! Please note:
  //! This mapping is only relevant for sequence commands.
  std::map<std::string, AbstractCmdGetterUPtr> cmd_getter_funcs_;

private:
  const std::string empty_str_ {};
  const pt::ptree::value_type empty_value_type_ {};
  const pt::ptree empty_tree_ {};

  const std::string XML_ATTR_STR {"<xmlattr>"};
  const std::string JOINT_STR {"joints"};
  const std::string POSE_STR {"pos"};
  const std::string XYZ_QUAT_STR {"xyzQuat"};
  const std::string XYZ_EULER_STR {"xyzEuler"};
  const std::string SEED_STR {"seed"};

  const std::string PTP_STR {"ptp"};
  const std::string LIN_STR {"lin"};
  const std::string CIRC_STR {"circ"};
  const std::string BLEND_STR {"blend"};

  const std::string PLANNING_GROUP_STR {"planningGroup"};
  const std::string TARGET_LINK_STR {"targetLink"};
  const std::string START_POS_STR {"startPos"};
  const std::string END_POS_STR {"endPos"};
  const std::string INTERMEDIATE_POS_STR {"intermediatePos"};
  const std::string CENTER_POS_STR {"centerPos"};
  const std::string VEL_STR {"vel"};
  const std::string ACC_STR {"acc"};


  const std::string POSES_PATH_STR {"testdata.poses"};
  const std::string PTPS_PATH_STR {"testdata." + PTP_STR + "s"};
  const std::string LINS_PATH_STR {"testdata."  + LIN_STR + "s"};
  const std::string CIRCS_PATH_STR {"testdata."  + CIRC_STR + "s"};
  const std::string SEQUENCE_PATH_STR {"testdata.sequences"};

  const std::string NAME_PATH_STR {XML_ATTR_STR + ".name"};
  const std::string CMD_TYPE_PATH_STR {XML_ATTR_STR + ".type"};
  const std::string BLEND_RADIUS_PATH_STR {XML_ATTR_STR + ".blend_radius"};
  const std::string LINK_NAME_PATH_STR {XML_ATTR_STR + ".link_name"};
  const std::string GROUP_NAME_PATH_STR {XML_ATTR_STR + ".group_name"};

};

void XmlTestdataLoader::strVec2doubleVec(std::vector<std::string> &strVec, std::vector<double> &dVec)
{
  dVec.resize(strVec.size());
  std::transform(strVec.begin(), strVec.end(), dVec.begin(), [](const std::string& val)
  {
    return std::stod(val);
  });
}

std::vector<double> XmlTestdataLoader::strVec2doubleVec(std::vector<std::string> &strVec)
{
  std::vector<double> vec;

  vec.resize(strVec.size());
  std::transform(strVec.begin(), strVec.end(), vec.begin(), [](const std::string& val)
  {
    return std::stod(val);
  });

  return vec;
}

using XmlTestDataLoaderUPtr = std::unique_ptr<TestdataLoader>;

}

#endif // XML_TESTDATA_LOADER_H
