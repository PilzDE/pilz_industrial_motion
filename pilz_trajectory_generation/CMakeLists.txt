cmake_minimum_required(VERSION 2.8.3)
project(pilz_trajectory_generation)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wno-unused-parameter)
add_definitions(-Werror)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pilz_msgs
  pilz_extensions
  pluginlib
  roscpp
  tf2
  tf2_eigen
  tf2_geometry_msgs
  eigen_conversions
  moveit_ros_move_group
  moveit_ros_planning_interface
  pilz_industrial_motion_testutils
  pilz_testutils
)



find_package(orocos_kdl)
find_package(Boost REQUIRED COMPONENTS )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  moveit_msgs
  pilz_msgs
  tf2_geometry_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/pilz_command_planner.cpp
  src/planning_context_loader.cpp
  src/joint_limits_validator.cpp
  src/joint_limits_aggregator.cpp
  src/joint_limits_container.cpp
  src/cartesian_limits_aggregator.cpp
  src/cartesian_limit.cpp
  src/limits_container.cpp
  src/trajectory_functions.cpp
  src/trajectory_appender.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Plugins ##
#############
add_library(pilz_command_planner
            src/pilz_command_planner.cpp
            src/planning_context_loader.cpp
            src/joint_limits_aggregator.cpp
            src/joint_limits_container.cpp
            src/limits_container.cpp
            src/cartesian_limit.cpp
            src/cartesian_limits_aggregator.cpp
            )
target_link_libraries(pilz_command_planner
                      ${catkin_LIBRARIES})

add_library(planning_context_loader_ptp
            src/planning_context_loader_ptp.cpp
            src/planning_context_loader.cpp
            src/trajectory_functions.cpp
            src/trajectory_generator.cpp
            src/trajectory_generator_ptp.cpp
            src/velocity_profile_atrap.cpp
            src/joint_limits_container.cpp
            )

target_link_libraries(planning_context_loader_ptp
                      ${catkin_LIBRARIES}) # DO NOT LINK ${PROJECT_NAME} here!

add_library(planning_context_loader_lin
            src/planning_context_loader_lin.cpp
            src/planning_context_loader.cpp
            src/trajectory_functions.cpp
            src/trajectory_generator.cpp
            src/trajectory_generator_lin.cpp
            src/velocity_profile_atrap.cpp
            )

target_link_libraries(planning_context_loader_lin
                      ${catkin_LIBRARIES}) # DO NOT LINK ${PROJECT_NAME} here!

add_library(planning_context_loader_circ
            src/planning_context_loader_circ.cpp
            src/planning_context_loader.cpp
            src/trajectory_functions.cpp
            src/trajectory_generator.cpp
            src/trajectory_generator_circ.cpp
            src/path_circle_generator.cpp
            )


target_link_libraries(planning_context_loader_circ
                      ${catkin_LIBRARIES}) # DO NOT LINK ${PROJECT_NAME} here!

add_library(command_list_manager
            src/command_list_manager.cpp
            src/trajectory_appender.cpp)
target_link_libraries(command_list_manager
            ${catkin_LIBRARIES})
add_dependencies(command_list_manager
            ${catkin_EXPORTED_TARGETS})

add_library(sequence_capability
            src/move_group_sequence_action.cpp
            src/move_group_sequence_service.cpp
            src/command_list_manager.cpp
            src/trajectory_blender_transition_window.cpp
            src/joint_limits_aggregator.cpp  # do we need joint limits and cartesian_limit here?
            src/joint_limits_container.cpp
            src/limits_container.cpp
            src/cartesian_limit.cpp
            src/cartesian_limits_aggregator.cpp
            src/trajectory_appender.cpp
            )
target_link_libraries(sequence_capability
                      ${catkin_LIBRARIES}) # DO NOT LINK ${PROJECT_NAME} here!
add_dependencies(sequence_capability
           ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
install(FILES
       plugins/sequence_capability_plugin_description.xml
       plugins/pilz_command_planner_plugin_description.xml
       plugins/planning_context_plugin_description.xml
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
       )

## Mark libraries for installation
install(TARGETS
   pilz_command_planner
   planning_context_loader_ptp
   planning_context_loader_lin
   planning_context_loader_circ
   command_list_manager
   sequence_capability
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(code_coverage REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    include(CodeCoverage)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  ## Declare a C++ library
  add_library(${PROJECT_NAME}_test
      test/test_utils.cpp
      src/trajectory_functions.cpp
      src/joint_limits_aggregator.cpp
      src/joint_limits_validator.cpp
      src/trajectory_generator.cpp
      src/trajectory_generator_ptp.cpp
      src/trajectory_generator_lin.cpp
      src/trajectory_generator_circ.cpp
      src/path_circle_generator.cpp
      src/velocity_profile_atrap.cpp
      src/limits_container.cpp
      src/joint_limits_container.cpp
      src/cartesian_limit.cpp
      test/motion_plan_request_builder.cpp
      test/motion_sequence_request_builder.cpp
      src/command_list_manager.cpp
      src/trajectory_blender_transition_window.cpp
      src/cartesian_limits_aggregator.cpp
      src/planning_context_loader.cpp
      src/trajectory_appender.cpp
  )

  target_link_libraries(${PROJECT_NAME}_test
    ${catkin_LIBRARIES}
  )

add_dependencies(${PROJECT_NAME}_test
            ${catkin_EXPORTED_TARGETS})

#########################
####Integration Tests####
#########################

# Planning Integration tests
add_rostest_gmock(integrationtest_sequence_action_capability
  test/integrationtest_sequence_action_capability.test
  test/integrationtest_sequence_action_capability.cpp
)

target_link_libraries(integrationtest_sequence_action_capability
   ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

add_rostest_gmock(integrationtest_sequence_action_capability_with_gripper
  test/integrationtest_sequence_action_capability_with_gripper.test
  test/integrationtest_sequence_action_capability.cpp
)
target_link_libraries(integrationtest_sequence_action_capability_with_gripper
   ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

add_rostest_gmock(integrationtest_sequence_action_capability_abb_irb2400
  test/integrationtest_sequence_action_capability_abb_irb2400.test
  test/integrationtest_sequence_action_capability.cpp
)
target_link_libraries(integrationtest_sequence_action_capability_abb_irb2400
  ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

add_rostest_gmock(integrationtest_sequence_action_capability_frankaemika_panda
  test/integrationtest_sequence_action_capability_frankaemika_panda.test
  test/integrationtest_sequence_action_capability.cpp
)
target_link_libraries(integrationtest_sequence_action_capability_frankaemika_panda
  ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

add_rostest_gtest(integrationtest_sequence_service_capability
  test/integrationtest_sequence_service_capability.test
  test/integrationtest_sequence_service_capability.cpp
)
target_link_libraries(integrationtest_sequence_service_capability
  ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

add_rostest(test/integrationtest_sequence_service_capability_frankaemika_panda.test
  DEPENDENCIES integrationtest_sequence_service_capability )

add_rostest(test/integrationtest_sequence_service_capability_abb_irb2400.test
  DEPENDENCIES integrationtest_sequence_service_capability )

add_rostest_gtest(integrationtest_sequence_service_capability_with_gripper
  test/integrationtest_sequence_service_capability_with_gripper.test
  test/integrationtest_sequence_service_capability.cpp
)
target_link_libraries(integrationtest_sequence_service_capability_with_gripper
  ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

# Planning Integration tests
add_rostest_gtest(integrationtest_command_planning
  test/integrationtest_command_planning.test
  test/integrationtest_command_planning.cpp
)
target_link_libraries(integrationtest_command_planning
   ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

add_rostest(test/integrationtest_command_planning_with_gripper.test
  DEPENDENCIES integrationtest_command_planning )

add_rostest(test/integrationtest_command_planning_abb_irb2400.test
  DEPENDENCIES integrationtest_command_planning )

add_rostest(test/integrationtest_command_planning_frankaemika_panda.test
  DEPENDENCIES integrationtest_command_planning )

# Blending Integration tests
add_rostest_gtest(integrationtest_command_list_manager
  test/integrationtest_command_list_manager.test
  test/integrationtest_command_list_manager.cpp
)
target_link_libraries(integrationtest_command_list_manager
${catkin_LIBRARIES} ${PROJECT_NAME}_test)


##################
####Unit Tests####
##################

  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(unittest_pilz_command_planner_direct
                   test/unittest_pilz_command_planner_direct.cpp
                   src/planning_context_loader_ptp.cpp)
  target_link_libraries(unittest_pilz_command_planner_direct
    ${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(unittest_velocity_profile_atrap
                   test/unittest_velocity_profile_atrap.cpp)
  target_link_libraries(unittest_velocity_profile_atrap
    ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # Trajectory Generator Unit Test
  add_rostest_gtest(unittest_trajectory_functions
    test/unittest_trajectory_functions.test
    test/unittest_trajectory_functions.cpp
  )

  target_link_libraries(unittest_trajectory_functions
    ${catkin_LIBRARIES}  ${PROJECT_NAME}_test)


  # unittest for trajectory blender transition window
  add_rostest_gtest(unittest_trajectory_blender_transition_window
    test/unittest_trajectory_blender_transition_window.test
    test/unittest_trajectory_blender_transition_window.cpp
  )

  target_link_libraries(unittest_trajectory_blender_transition_window
    ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # trajectory generator Unit Test
  add_rostest_gtest(unittest_trajectory_generator_common
    test/unittest_trajectory_generator_common.test
    test/unittest_trajectory_generator_common.cpp
  )

  target_link_libraries(unittest_trajectory_generator_common
      ${catkin_LIBRARIES} ${PROJECT_NAME}_test)


  # trajectory generator circ Unit Test
  add_rostest_gtest(unittest_trajectory_generator_circ
    test/unittest_trajectory_generator_circ.test
    test/unittest_trajectory_generator_circ.cpp
  )

  target_link_libraries(unittest_trajectory_generator_circ
    ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # trajectory generator lin Unit Test
  add_rostest_gtest(unittest_trajectory_generator_lin
    test/unittest_trajectory_generator_lin.test
    test/unittest_trajectory_generator_lin.cpp
  )

  target_link_libraries(unittest_trajectory_generator_lin
      ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # trajectory generator ptp Unit Test
  add_rostest_gtest(unittest_trajectory_generator_ptp
    test/unittest_trajectory_generator_ptp.test
    test/unittest_trajectory_generator_ptp.cpp
  )

  target_link_libraries(unittest_trajectory_generator_ptp
    ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # Command Planner Unit Test
  add_rostest_gtest(unittest_pilz_command_planner
    test/unittest_pilz_command_planner.test
    test/unittest_pilz_command_planner.cpp
  )

  target_link_libraries(unittest_pilz_command_planner
    ${catkin_LIBRARIES})

  # JointLimitsAggregator Unit Test
  add_rostest_gtest(unittest_joint_limits_aggregator
    test/unittest_joint_limits_aggregator.test
    test/unittest_joint_limits_aggregator.cpp
  )

  target_link_libraries(unittest_joint_limits_aggregator
      ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # JointLimitsContainer Unit Test
  catkin_add_gtest(unittest_joint_limits_container
    test/unittest_joint_limits_container.cpp
  )

  target_link_libraries(unittest_joint_limits_container
    ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # JointLimitsValidator Unit Test
  catkin_add_gtest(unittest_joint_limits_validator
    test/unittest_joint_limits_validator.cpp
  )

  target_link_libraries(unittest_joint_limits_validator
     ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

   # Cartesian Limits Aggregator Unit Test
  add_rostest_gtest(unittest_cartesian_limits_aggregator
    test/unittest_cartesian_limits_aggregator.test
    test/unittest_cartesian_limits_aggregator.cpp
  )

  target_link_libraries(unittest_cartesian_limits_aggregator
    ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # PlanningContextLoaderPTP Unit Test
  add_rostest_gtest(unittest_planning_context_loaders
    test/unittest_planning_context_loaders.test
    test/unittest_planning_context_loaders.cpp
  )

  target_link_libraries(unittest_planning_context_loaders
    ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # PlanningContext Unit Test (Typed test)
  add_rostest_gtest(unittest_planning_context
    test/unittest_planning_context.test
    test/unittest_planning_context.cpp
    )

  target_link_libraries(unittest_planning_context
     ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # RobotTrajectoryHelper Unit Test
  add_rostest_gtest(unittest_trajectory_appender
    test/unittest_trajectory_appender.test
    test/unittest_trajectory_appender.cpp
    src/trajectory_appender.cpp)

  target_link_libraries(unittest_trajectory_appender
    ${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_test)

  # to run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()
endif()
