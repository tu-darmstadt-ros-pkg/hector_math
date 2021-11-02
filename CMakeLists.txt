cmake_minimum_required(VERSION 3.5.1)
project(hector_math VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS urdf)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hector_math
  CATKIN_DEPENDS urdf
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
  add_executable(benchmark_robot_model benchmark/robot_model.cpp)
  target_link_libraries(benchmark_robot_model ${PROJECT_NAME} ${catkin_LIBRARIES} benchmark pthread)

  # Iterator benchmark
  add_executable(benchmark_iterators benchmark/iterators.cpp)
  find_package(grid_map_core QUIET)
  if (${grid_map_core_FOUND})
    add_definitions(-DBENCHMARK_ENABLE_GRIDMAP)
    include_directories(${grid_map_core_INCLUDE_DIRS})
    target_link_libraries(benchmark_iterators ${grid_map_core_LIBRARIES})
  endif()
  target_link_libraries(benchmark_iterators ${PROJECT_NAME} ${catkin_LIBRARIES} benchmark pthread)

  add_executable(show_iterators benchmark/show_iterators.cpp)
  target_link_libraries(show_iterators ${PROJECT_NAME} ${catkin_LIBRARIES})
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

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roscpp REQUIRED)
  find_package(rostest REQUIRED)

  include_directories(include ${catkin_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS} ${rostest_INCLUDE_DIRS})
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test test/test_hector_math.cpp)
 target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})

endif()
