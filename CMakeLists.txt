cmake_minimum_required(VERSION 3.10)
project(3d_telemetry)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
add_subdirectory(src)

add_executable(3d_telemetry main.cpp)
target_link_libraries(3d_telemetry 3dtelemetry)