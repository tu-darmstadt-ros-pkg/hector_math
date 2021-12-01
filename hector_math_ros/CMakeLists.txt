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
  target_link_libraries(benchmark_robot_model ${PROJECT_NAME} ${catkin_LIBRARIES} benchmark pthread)
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


