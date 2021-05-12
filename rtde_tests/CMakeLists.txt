cmake_minimum_required(VERSION 3.5)
project(rtde_test)


find_package(ur_rtde REQUIRED)
add_executable(rtde_test rtde_test.cpp)
target_link_libraries(rtde_test PRIVATE ur_rtde::rtde)
