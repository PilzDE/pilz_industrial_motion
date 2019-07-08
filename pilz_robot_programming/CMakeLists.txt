cmake_minimum_required(VERSION 2.8.3)
project(pilz_robot_programming)

find_package(catkin REQUIRED COMPONENTS roslint rospy)

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

function(ADD_PYTHON_COVERAGE)
    set(options NONE)
    set(oneValueArgs NAME)
    set(multiValueArgs DEPENDENCIES)
    cmake_parse_arguments(Coverage "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_custom_target(${Coverage_NAME}_cleanup
      # Cleanup coverage data from previous run
      COMMAND python-coverage erase
      WORKING_DIRECTORY $ENV{HOME}/.ros
      COMMENT "Resetting python code coverage counters to zero."
  )

  add_custom_target(${Coverage_NAME}
      COMMAND python-coverage combine
      COMMAND python-coverage report --include "*${PROJECT_SOURCE_DIR}*" --omit ${COVERAGE_EXCLUDES}
      COMMAND python-coverage html --include "*${PROJECT_SOURCE_DIR}*" --omit ${COVERAGE_EXCLUDES}
      WORKING_DIRECTORY $ENV{HOME}/.ros
      DEPENDS _run_tests_${PROJECT_NAME}
      COMMENT "Processing code coverage counters and generating report. Please find the generated report in $ENV{HOME}/.ros/htmlcov/index.html"
  )

  add_dependencies(_run_tests_${PROJECT_NAME} ${Coverage_NAME}_cleanup)
endfunction() # SETUP_TARGET_FOR_COVERAGE

  # to run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage
  # Import errors may arise when packages are installed in the non-standard path /usr/lib/python2.7/dist-packages
  # This problem can be avoided by including the path in the PYTHONPATH environment variable
  if(ENABLE_COVERAGE_TESTING)
      set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*") #comma-sparated list of ignored patterns
      add_python_coverage(
          NAME ${PROJECT_NAME}_coverage
          )
  endif(ENABLE_COVERAGE_TESTING)

endif()
