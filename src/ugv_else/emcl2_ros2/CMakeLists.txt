cmake_minimum_required(VERSION 3.8)
project(emcl2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(emcl2_node
  src/Mcl.cpp
  src/ExpResetMcl2.cpp
  src/Particle.cpp
  src/OdomModel.cpp
  src/Pose.cpp
  src/Scan.cpp
  src/LikelihoodFieldMap.cpp
  src/emcl2_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  test/test.bash
  test/test_gui.bash
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  test
)
