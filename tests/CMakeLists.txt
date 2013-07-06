
add_executable(test_emit_yaml test_emit_yaml.cpp)
target_link_libraries(test_emit_yaml yaml-cpp ${PCL_LIBRARIES})

add_executable(filter test_filter.cpp)
target_link_libraries(filter ${PCL_LIBRARIES})

add_executable(test_sf test_sf.cpp)
target_link_libraries(test_sf ${PCL_LIBRARIES} ${CERES_LIBRARIES})

