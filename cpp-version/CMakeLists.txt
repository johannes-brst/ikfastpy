set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_minimum_required(VERSION 3.5)
project(cpp-version)

# add_subdirectory(ikfast_wrapper)
# add_subdirectory(ikfast)

find_package(BLAS)
find_package(LAPACK)
add_library(BLAS SHARED IMPORTED)
add_library(LAPACK SHARED IMPORTED)
if(LAPACK_FOUND AND BLAS_FOUND)
   # set(lapackblas_libraries "${BLAS_LIBRARIES} ${LAPACK_LIBRARIES}")
   set_target_properties(BLAS PROPERTIES IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/blas/libblas.so)
   set_target_properties(LAPACK PROPERTIES IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/lapack/liblapack.so)
endif()

add_library(Kinematics ikfast_wrapper.cpp)
target_link_libraries(Kinematics BLAS LAPACK) 

find_package(ur_rtde REQUIRED)

add_executable(demo demo.cpp)
add_executable(calculate_kinematics calculate_kinematics.cpp)

#target_link_libraries(ikfast_wrapper Kinematics ${lapackblas_libraries}) 
target_link_libraries(demo PRIVATE ur_rtde::rtde Kinematics) 
target_link_libraries(calculate_kinematics PRIVATE ur_rtde::rtde Kinematics) 

target_include_directories(demo PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          )
target_include_directories(calculate_kinematics PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          )

