cmake_minimum_required(VERSION 3.8)
project(tug_husky)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install config directory
install(DIRECTORY "config/" DESTINATION "share/${CMAKE_PROJECT_NAME}/config/")

# Install launch directory
install(DIRECTORY "launch/" DESTINATION "share/${CMAKE_PROJECT_NAME}/launch/")

# Install map directory
install(DIRECTORY "map/" DESTINATION "share/${CMAKE_PROJECT_NAME}/map/")

# Install urdf directory
install(DIRECTORY "urdf/" DESTINATION "share/${CMAKE_PROJECT_NAME}/urdf/")

# Install world directory
install(DIRECTORY "world/" DESTINATION "share/${CMAKE_PROJECT_NAME}/world/")

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
