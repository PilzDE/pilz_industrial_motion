cmake_minimum_required(VERSION 2.8.3)
project(pilz_store_positions)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  std_msgs
  visualization_msgs
)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

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
    if(ENABLE_COVERAGE_TESTING)
      add_pytests(${file} OPTIONS "--cov=${PROJECT_NAME} --cov-append")
    else()
      add_pytests(${file})
    endif()
  endforeach()

  # check for pep8 compliance and fail test on error(s)
  roslint_python( )
  roslint_add_test()

  # Fasest way to generate coverage - Make sure to run the test in sequence:
  # catkin_make -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE_TESTING=ON tests
  # catkin_make -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE_TESTING=ON pilz_store_positions_coverage -j1
  # or
  # catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
  # catkin build pilz_store_positions -v --no-deps --catkin-make-args tests
  # catkin build pilz_store_positions -j 1 -v --no-deps --catkin-make-args pilz_store_positions_coverage
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      # specifying dependencies in a reliable way is on open issue
      # see https://github.com/mikeferguson/code_coverage/pull/14
      #DEPENDENCIES tests
    )
  endif()

endif()

