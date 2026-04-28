cmake_minimum_required(VERSION 3.10)
project(hector_math VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ament_cmake QUIET)
find_package(catkin QUIET)

if (catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hector_math
    DEPENDS Eigen3
  )
endif()

###########
## Build ##
###########

add_library(hector_math INTERFACE)
add_library(hector_math::hector_math ALIAS hector_math)
target_include_directories(hector_math INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hector_math INTERFACE Eigen3::Eigen)

if (BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif (BUILD_TESTING)

find_package(benchmark QUIET)
if (${benchmark_FOUND})
  message(STATUS "benchmark found. Building benchmarks.")

  add_executable(benchmark_caches benchmark/caches.cpp)
  target_link_libraries(benchmark_caches PRIVATE hector_math benchmark benchmark_main pthread)

  add_executable(quaternion_binning_modes benchmark/quaternion_binning_modes.cpp)
  target_link_libraries(quaternion_binning_modes PRIVATE hector_math benchmark benchmark_main pthread)

  add_executable(benchmark_fit_plane benchmark/fit_plane.cpp)
  target_link_libraries(benchmark_fit_plane PRIVATE hector_math benchmark benchmark_main pthread)

  add_executable(benchmark_search benchmark/search.cpp)
  target_link_libraries(benchmark_search PRIVATE hector_math benchmark benchmark_main pthread)

  # Show iterators
  add_executable(show_iterators scripts/show_iterators.cpp)
  target_link_libraries(show_iterators PRIVATE hector_math)

  # Iterator benchmark
  add_executable(benchmark_iterators benchmark/iterators.cpp)
  # Disabled until grid_map_core fixes their pollution of the definitions
  #find_package(grid_map_core QUIET)
  if (${grid_map_core_FOUND})
    target_compile_definitions(benchmark_iterators PRIVATE BENCHMARK_ENABLE_GRIDMAP)
    target_link_libraries(benchmark_iterators PRIVATE grid_map_core::grid_map_core)
  endif()
  target_link_libraries(benchmark_iterators PRIVATE hector_math benchmark benchmark_main pthread)

  install(TARGETS benchmark_caches benchmark_fit_plane benchmark_search quaternion_binning_modes show_iterators benchmark_iterators
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "benchmark library not found. Not building benchmarks.")
endif()

#############
## Install ##
#############

install(TARGETS hector_math EXPORT hector_math-targets)
install(EXPORT hector_math-targets
  FILE hector_math-targets.cmake
  NAMESPACE hector_math::
  DESTINATION lib/cmake/hector_math
)

install(DIRECTORY include/ DESTINATION include)

if (ament_cmake_FOUND)
  ament_export_targets(hector_math-targets)
  ament_export_dependencies(Eigen3)
  ament_export_include_directories(include)
  ament_package()
endif()
