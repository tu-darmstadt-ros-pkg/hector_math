cmake_minimum_required(VERSION 3.16)
project(hector_math_ros VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hector_math REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

###########
## Build ##
###########


add_library(hector_math_ros INTERFACE)
add_library(hector_math_ros::hector_math_ros ALIAS hector_math_ros)
target_include_directories(hector_math_ros INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hector_math_ros INTERFACE
                      hector_math::hector_math
                      rclcpp::rclcpp
                      sensor_msgs::sensor_msgs
                      urdf::urdf)
target_link_libraries(hector_math_ros INTERFACE hector_math::hector_math rclcpp::rclcpp urdf::urdf)

if (BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif (BUILD_TESTING)

find_package(benchmark QUIET)
if (${benchmark_FOUND})
  add_executable(benchmark_robot_model benchmark/robot_model.cpp)
  target_link_libraries(benchmark_robot_model PUBLIC ament_index_cpp::ament_index_cpp hector_math_ros benchmark benchmark_main pthread)
endif()

#############
## Install ##
#############

install(TARGETS hector_math_ros EXPORT hector_math_ros-targets)
install(EXPORT hector_math_ros-targets
  FILE hector_math_ros-targets.cmake
  NAMESPACE hector_math_ros::
  DESTINATION lib/cmake/hector_math_ros
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(hector_math_ros-targets)
ament_export_dependencies(hector_math urdf)
ament_export_include_directories(include)
ament_package()
