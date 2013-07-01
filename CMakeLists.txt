cmake_minimum_required (VERSION 2.6)
project (lollipop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

find_package(PCL 1.5 REQUIRED PATHS /usr NO_DEFAULT_PATH)

include_directories(
  ${PCL_INCLUDE_DIRS}
   /usr/local/include/yaml-cpp/
  .)

link_directories(
  /usr/local/lib/
  ${PCL_LIBRARY_DIRS}
  )

add_definitions(${PCL_DEFINITIONS})

add_subdirectory(tests)
add_subdirectory(src)
