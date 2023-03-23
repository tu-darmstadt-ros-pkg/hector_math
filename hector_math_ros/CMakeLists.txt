cmake_minimum_required(VERSION 3.5.1)
project(hector_math_ros VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS hector_math urdf)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hector_math_ros
  CATKIN_DEPENDS hector_math urdf
)

###########
## Build ##
###########


add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} INTERFACE ${EIGEN3_LIBRARIES})

find_package(benchmark QUIET)
if (${benchmark_FOUND})
  add_executable(benchmark_robot_model benchmark/robot_model.cpp)
  target_link_libraries(benchmark_robot_model ${PROJECT_NAME} ${catkin_LIBRARIES} benchmark benchmark_main pthread)
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roscpp REQUIRED)
  find_package(rostest REQUIRED)
  find_package(roslib REQUIRED) # Provides ros::package
  set(TEST_LIBRARIES ${PROJECT_NAME} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${roslib_INCLUDE_DIRS}
    ${rostest_INCLUDE_DIRS}
  )
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(test_joint_state_subscriber test/test_joint_state_subscriber.cpp)
  target_link_libraries(test_joint_state_subscriber ${TEST_LIBRARIES})
endif ()

# to run: catkin build --this --no-deps -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -v --catkin-make-args hector_math_coverage
# Path to results overview will be printed in the build process
# Big thanks to the moveit people from whose docs I've obtained the information on how to get the coverage
if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
  include(CodeCoverage)
  APPEND_COVERAGE_COMPILER_FLAGS()
  set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/build/*")
  add_code_coverage(NAME ${PROJECT_NAME}_coverage)
endif ()

