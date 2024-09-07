cmake_minimum_required(VERSION 3.5)

project(ugv_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(DIRECTORY launch meshes urdf rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rosidl_default_generators)

ament_package()

