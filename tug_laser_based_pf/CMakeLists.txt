cmake_minimum_required(VERSION 3.8)
project(tug_laser_based_pf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs)

# Laser based pf node
add_executable(tug_laser_based_pf "src/tug_laser_based_pf.cpp" "src/LaserBasedPfNode.cpp" "src/Particle.cpp")
target_compile_features(tug_laser_based_pf PUBLIC cxx_std_17)
target_include_directories(tug_laser_based_pf PUBLIC "include/")
ament_target_dependencies(
  tug_laser_based_pf
    Eigen3
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# Install binaries
install(TARGETS tug_laser_based_pf DESTINATION "lib/${CMAKE_PROJECT_NAME}")

# Install directories
install(DIRECTORY "launch/" DESTINATION "share/${CMAKE_PROJECT_NAME}/launch/")
install(DIRECTORY "rviz/" DESTINATION "share/${CMAKE_PROJECT_NAME}/rviz/")

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
