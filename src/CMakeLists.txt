
add_executable(show_pcd show_pcd.cpp)
target_link_libraries(show_pcd ${PCL_LIBRARIES} yaml-cpp)
