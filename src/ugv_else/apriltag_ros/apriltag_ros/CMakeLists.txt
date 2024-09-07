cmake_minimum_required(VERSION 3.5)

project(apriltag_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(apriltag REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(tag_detector src/AprilTagNode.cpp)

add_library(AprilTagNode SHARED src/AprilTagNode.cpp)

set(DEPENDENCIES
    "rclcpp"
    "rclcpp_components"
    "sensor_msgs"
    "tf2_msgs"
    "tf2_ros"
    "image_transport"
    "image_geometry"
    "cv_bridge"
    "geometry_msgs"
    "apriltag_msgs"
    "Eigen3"
    "apriltag"
)

ament_target_dependencies(AprilTagNode ${DEPENDENCIES})
ament_target_dependencies(tag_detector ${DEPENDENCIES})

target_link_libraries(AprilTagNode apriltag::apriltag)
target_link_libraries(tag_detector apriltag::apriltag)

rclcpp_components_register_nodes(AprilTagNode "AprilTagNode")

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

install(TARGETS AprilTagNode
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(TARGETS tag_detector
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)


install(DIRECTORY cfg/ DESTINATION share/${PROJECT_NAME}/cfg)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${DEPENDENCIES})
ament_package()
