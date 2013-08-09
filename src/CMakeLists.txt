
add_executable(show_pcd show_pcd.cpp)
target_link_libraries(show_pcd ${PCL_LIBRARIES} yaml-cpp)

add_executable(sln2dat sln2dat.cpp)
target_link_libraries(sln2dat yaml-cpp)
