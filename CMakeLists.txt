cmake_minimum_required (VERSION 2.6)
project (lollipop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

include_directories(
   /usr/local/include/yaml-cpp/
  .)

link_directories(
  /usr/local/lib/
  )

add_subdirectory(tests)
