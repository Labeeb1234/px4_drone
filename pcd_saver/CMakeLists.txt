cmake_minimum_required(VERSION 3.8)
project(pcd_saver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(point_cloud_to_pcd src/pcd_saver.cpp)
ament_target_dependencies(point_cloud_to_pcd
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
  tf2_ros
)

install(
  TARGETS
    point_cloud_to_pcd
  DESTINATION lib/${PROJECT_NAME}
)


# rclcpp_components_register_node(
#     PointCloudToPCD_component
#     PLUGIN "PointCloudToPCD"
#     EXECUTABLE pcd_saver
# )

# ament_export_targets(export_PointCloudToPCD_component)
# install(TARGETS PointCloudToPCD_component
#         EXPORT export_PointCloudToPCD_component
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
