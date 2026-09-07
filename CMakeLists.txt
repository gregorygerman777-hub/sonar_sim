cmake_minimum_required(VERSION 3.20)
project(sonar_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(sonar_core core/geometry.cpp core/physics.cpp core/simulator.cpp core/camera.cpp core/waveform.cpp)
target_include_directories(sonar_core PUBLIC core)

find_package(Threads REQUIRED)
target_link_libraries(sonar_core PUBLIC Threads::Threads)
