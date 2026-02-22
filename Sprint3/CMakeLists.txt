cmake_minimum_required(VERSION 3.10)
project(CruiseControlSim VERSION 1.0 LANGUAGES CXX)

# Use C++17 for std::clamp and [[unlikely]] attributes found in your code
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include_directories(include)


set(SOURCES 
    src/main.cpp 
    src/CruiseControl.cpp
)

# Create the executable
add_executable(CruiseControlSim ${SOURCES})

# Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()