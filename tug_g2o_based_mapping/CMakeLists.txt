cmake_minimum_required(VERSION 3.8)
project(tug_g2o_based_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# add target
add_executable(tug_g2o_based_mapping "src/tug_g2o_based_mapping.cpp" "src/G2OBasedMapping.cpp")
target_compile_features(tug_g2o_based_mapping PUBLIC cxx_std_17)
target_include_directories(tug_g2o_based_mapping PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        /usr/include/suitesparse
)
ament_target_dependencies(
        tug_g2o_based_mapping
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

target_link_libraries(
        tug_g2o_based_mapping
        "g2o::core"
        "g2o::stuff"
        "g2o::types_data"
        "g2o::solver_csparse"
        "g2o::csparse_extension"
)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# install target
install(TARGETS tug_g2o_based_mapping DESTINATION "lib/${CMAKE_PROJECT_NAME}")

# install launch directory
install(DIRECTORY "launch/" DESTINATION "share/${CMAKE_PROJECT_NAME}/launch/")

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