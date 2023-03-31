cmake_minimum_required(VERSION 3.5.1)
project(hector_math VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hector_math
  CATKIN_DEPENDS
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} INTERFACE ${EIGEN3_LIBRARIES})


find_package(benchmark QUIET)
if (${benchmark_FOUND})
  message(STATUS "benchmark found. Building benchmarks.")
  # Iterator benchmark
  add_executable(benchmark_iterators benchmark/iterators.cpp)
  find_package(grid_map_core QUIET)
  if (${grid_map_core_FOUND})
    add_definitions(-DBENCHMARK_ENABLE_GRIDMAP)
    include_directories(${grid_map_core_INCLUDE_DIRS})
    target_link_libraries(benchmark_iterators ${grid_map_core_LIBRARIES})
  endif()
  target_link_libraries(benchmark_iterators ${PROJECT_NAME} ${catkin_LIBRARIES} benchmark benchmark_main pthread)

  add_executable(benchmark_caches benchmark/caches.cpp)
  target_link_libraries(benchmark_caches ${PROJECT_NAME} ${catkin_LIBRARIES} benchmark benchmark_main pthread)

  add_executable(quaternion_binning_modes benchmark/quaternion_binning_modes.cpp)
  target_link_libraries(quaternion_binning_modes ${PROJECT_NAME} ${catkin_LIBRARIES} benchmark benchmark_main pthread)

  add_executable(show_iterators scripts/show_iterators.cpp)
  target_link_libraries(show_iterators ${PROJECT_NAME} ${catkin_LIBRARIES})
else()
  message(WARN benchmark library not found. Not building benchmarks.)
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
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
          ${eigen_checks_INCLUDE_DIRS}
          ${catkin_INCLUDE_DIRS}
          ${EIGEN3_INCLUDE_DIRS}
          ${roscpp_INCLUDE_DIRS}
          ${roslib_INCLUDE_DIRS}
          ${rostest_INCLUDE_DIRS}
  )
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(test_bounded_vector test/test_bounded_vector.cpp)
  target_link_libraries(test_bounded_vector ${TEST_LIBRARIES})

  catkin_add_gtest(test_bounding_box test/test_bounding_box.cpp)
  target_link_libraries(test_bounding_box ${TEST_LIBRARIES})

  catkin_add_gtest(test_eigen_helper test/test_eigen_helper.cpp)
  target_link_libraries(test_eigen_helper ${TEST_LIBRARIES})

  catkin_add_gtest(test_hector_iterators test/test_hector_iterators.cpp)
  target_link_libraries(test_hector_iterators ${TEST_LIBRARIES})

  catkin_add_gtest(test_hector_map_operations test/test_hector_map_operations.cpp)
  target_link_libraries(test_hector_map_operations ${TEST_LIBRARIES})

  catkin_add_gtest(test_pose test/test_pose.cpp)
  target_link_libraries(test_pose ${TEST_LIBRARIES})

  catkin_add_gtest(test_quaternion_binning test/test_quaternion_binning.cpp)
  target_link_libraries(test_quaternion_binning ${TEST_LIBRARIES})

  catkin_add_gtest(test_ring_buffer test/test_ring_buffer.cpp)
  target_link_libraries(test_ring_buffer ${TEST_LIBRARIES})
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
