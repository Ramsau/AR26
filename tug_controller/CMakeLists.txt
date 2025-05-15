cmake_minimum_required(VERSION 3.8)
project(tug_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

# Add library
add_library(tug_controller SHARED "src/tug_controller.cpp")
target_compile_features(tug_controller PUBLIC cxx_std_17)
target_include_directories(tug_controller PUBLIC "include/")
ament_target_dependencies(tug_controller nav2_core pluginlib)

# Install directories
install(DIRECTORY "config/" DESTINATION "share/${CMAKE_PROJECT_NAME}/config/")
install(DIRECTORY "launch/" DESTINATION "share/${CMAKE_PROJECT_NAME}/launch/")
install(DIRECTORY "rviz/" DESTINATION "share/${CMAKE_PROJECT_NAME}/rviz/")

# Install targets
install(TARGETS tug_controller DESTINATION "lib/${CMAKE_PROJECT_NAME}/")

# Export plugin description
pluginlib_export_plugin_description_file(nav2_core "tug_controller_plugin.xml")

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
