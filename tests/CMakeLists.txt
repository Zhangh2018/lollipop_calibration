
#add_executable(test_emit_yaml test_emit_yaml.cpp)
#target_link_libraries(test_emit_yaml yaml-cpp ${PCL_LIBRARIES})

add_executable(load_yaml test_load_yaml.cpp)
target_link_libraries(load_yaml yaml-cpp)

add_executable(pipeline test_pipeline.cpp)
target_link_libraries(pipeline ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable(test_sf test_sf.cpp)
#target_link_libraries(test_sf ${PCL_LIBRARIES} ${CERES_LIBRARIES})

