cmake_minimum_required(VERSION 2.8.3)
project(pilz_store_positions)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  std_msgs
  visualization_msgs
)

catkin_package()

#############
## Install ##
#############

catkin_python_setup()

catkin_install_python(PROGRAMS
   scripts/pose_visualisation_node
   scripts/store_current_pose
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_pytest REQUIRED)

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  file(GLOB integrationtest_files "test/integrationtests/*.test")
  foreach(file ${integrationtest_files})
    if(ENABLE_COVERAGE_TESTING)
        add_rostest(${file} ARGS coverage:=TRUE)
    else()
        add_rostest(${file})
    endif()
  endforeach()

  file(GLOB unittest_files "test/unittests/test_*.py")
  foreach(file ${unittest_files})
    add_pytests(${file})
  endforeach()

  # check for pep8 compliance and fail test on error(s)
  roslint_python( )
  roslint_add_test()

endif()

