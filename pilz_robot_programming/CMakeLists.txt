cmake_minimum_required(VERSION 2.8.3)
project(pilz_robot_programming)

find_package(catkin REQUIRED COMPONENTS roslint rospy)

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
   examples/demo_program.py
   examples/demo_gripper_program.py
   examples/demo_brake_test_program.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  # integration test files
  file(GLOB integrationtest_files "test/integrationtests/*.test")
  file(GLOB integrationtest_scripts "test/integrationtests/*.py")

  if(ENABLE_COVERAGE_TESTING)
    foreach(file ${integrationtest_files})
      add_rostest(${file} ARGS coverage:=True)
    endforeach()
  else()
    foreach(file ${integrationtest_files})
      add_rostest(${file})
    endforeach()
  endif()

  roslint_python(
    src/${PROJECT_NAME}/__init__.py
    src/${PROJECT_NAME}/robot.py
    src/${PROJECT_NAME}/commands.py
    src/${PROJECT_NAME}/exceptions.py
    src/${PROJECT_NAME}/move_control_request.py
    ${integrationtest_scripts}
  )
  roslint_add_test()

  # to run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage
  # Import errors may arise when packages are installed in the non-standard path /usr/lib/python2.7/dist-packages
  # This problem can be avoided by including the path in the PYTHONPATH environment variable
  if(ENABLE_COVERAGE_TESTING)
      set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*") #comma-sparated list of ignored patterns
      add_code_coverage(
          NAME ${PROJECT_NAME}_coverage
          )
  endif(ENABLE_COVERAGE_TESTING)
endif()
