
#add_executable(test_emit_yaml test_emit_yaml.cpp)
#target_link_libraries(test_emit_yaml yaml-cpp ${PCL_LIBRARIES})

#add_executable(load_yaml test_load_yaml.cpp)
#target_link_libraries(load_yaml yaml-cpp)

#add_executable(pipeline test_pipeline.cpp)
#target_link_libraries(pipeline ${PCL_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)

add_executable(detect_ball test_detect_ball.cpp)
target_link_libraries(detect_ball ${PCL_LIBRARIES}  ${CERES_LIBRARIES} yaml-cpp)

add_executable(solveTf test_tfsolver.cpp)
target_link_libraries(solveTf ${PCL_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)

add_executable(run2nd test_2ndpass.cpp)
target_link_libraries(run2nd ${PCL_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)

add_executable(show_result show_result.cpp)
target_link_libraries(show_result ${PCL_LIBRARIES} yaml-cpp)

#add_executable(test_sf test_sf.cpp)
#target_link_libraries(test_sf ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(raycost test_cost.cpp)
target_link_libraries(raycost ${CERES_LIBRARIES})

#add_executable(error_analysis error_analysis.cpp)
#target_link_libraries(error_analysis ${PCL_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)
