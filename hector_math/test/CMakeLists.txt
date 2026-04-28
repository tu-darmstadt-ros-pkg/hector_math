find_package(GTest REQUIRED)
include(CTest)
include(GoogleTest)

add_executable(test_bounded_vector test_bounded_vector.cpp)
target_link_libraries(test_bounded_vector GTest::gtest_main ${PROJECT_NAME})
gtest_discover_tests(test_bounded_vector)

add_executable(test_bounding_box test_bounding_box.cpp)
target_link_libraries(test_bounding_box GTest::gtest_main ${PROJECT_NAME})
gtest_discover_tests(test_bounding_box)

add_executable(test_eigen_helper test_eigen_helper.cpp)
target_link_libraries(test_eigen_helper GTest::gtest_main ${PROJECT_NAME})
gtest_discover_tests(test_eigen_helper)

add_executable(test_hector_iterators test_hector_iterators.cpp)
target_link_libraries(test_hector_iterators GTest::gtest_main ${PROJECT_NAME})
gtest_discover_tests(test_hector_iterators)

add_executable(test_hector_map_operations test_hector_map_operations.cpp)
target_link_libraries(test_hector_map_operations GTest::gtest_main ${PROJECT_NAME})
gtest_discover_tests(test_hector_map_operations)

add_executable(test_pose test_pose.cpp)
target_link_libraries(test_pose GTest::gtest_main ${PROJECT_NAME})
gtest_discover_tests(test_pose)

add_executable(test_quaternion_binning test_quaternion_binning.cpp)
target_link_libraries(test_quaternion_binning GTest::gtest_main ${PROJECT_NAME})
gtest_discover_tests(test_quaternion_binning)

add_executable(test_ring_buffer test_ring_buffer.cpp)
target_link_libraries(test_ring_buffer GTest::gtest_main ${PROJECT_NAME})
gtest_discover_tests(test_ring_buffer)

add_executable(test_search test_search.cpp)
target_link_libraries(test_search GTest::gtest_main ${PROJECT_NAME})
gtest_discover_tests(test_search)
