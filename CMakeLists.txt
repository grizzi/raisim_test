cmake_minimum_required(VERSION 3.5)
project(raisim_test LANGUAGES CXX)

#==
# Dependencies
#==

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(OpenMP REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}  -Wall")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories ( ${EIGEN3_INCLUDE_DIRS} )

add_executable(raisim_test src/raisim_test.cpp)
target_link_libraries(raisim_test PUBLIC raisim::raisim pthread)
target_include_directories(raisim_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

