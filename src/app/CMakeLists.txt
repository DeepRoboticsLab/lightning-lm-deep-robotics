foreach(app run_slam_offline run_slam_online run_loc_offline run_loc_online)
    add_executable(${app} ${app}.cc)
    target_link_libraries(${app} ${PROJECT_NAME}.libs ${third_party_libs})
    install(TARGETS ${app} DESTINATION lib/${PROJECT_NAME})
endforeach()

# Saved PCD display is independent of the four estimator applications.
add_executable(publish_map publish_map.cc)
ament_target_dependencies(publish_map rclcpp sensor_msgs tf2_msgs)
target_link_libraries(publish_map ${PCL_LIBRARIES})
install(TARGETS publish_map DESTINATION lib/${PROJECT_NAME})

# Use installed RViz displays with controlled render-timer shutdown over X11.
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
add_executable(run_rviz run_rviz.cc)
ament_target_dependencies(run_rviz rviz_common rclcpp)
target_link_libraries(run_rviz Qt5::Widgets)
install(TARGETS run_rviz DESTINATION lib/${PROJECT_NAME})
