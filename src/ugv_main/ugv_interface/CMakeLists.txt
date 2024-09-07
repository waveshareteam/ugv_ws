cmake_minimum_required(VERSION 3.5)
project(ugv_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MapSave.srv"
  "action/Behavior.action"
)
#ament_target_dependencies(
#  "rclcpp"
#  "std_msgs"
#  "cv_bridge"
#  "sensor_msgs"
#  "image_transport"
#  "tf2_ros"
#  "message_filters"
#  "geometry_msgs"
#)


#install(TARGETS 
#  DESTINATION lib/${PROJECT_NAME})

include_directories(include/ugv_slam)
install(DIRECTORY srv action DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#target_link_libraries(pointcloud_mapping ${PCL_LIBRARIES})
ament_package()
