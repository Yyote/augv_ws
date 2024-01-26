cmake_minimum_required(VERSION 3.8)
project(ackerman_regulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(regulators_lib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(euler_angles_lib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(augv_navigation_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(deps ament_cmake rclcpp regulators_lib geometry_msgs euler_angles_lib augv_navigation_msgs tf2_geometry_msgs tf2 visualization_msgs)

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


add_executable(ackerman_regulator_node src/ackerman_regulator.cpp) 
target_link_libraries(ackerman_regulator_node regulators_lib::regulators_lib euler_angles_lib::euler_angles)
ament_target_dependencies(ackerman_regulator_node ${deps})

# install(
#   DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include
# )

install(
  TARGETS ackerman_regulator_node
  EXPORT ackerman_regulator_node
  LIBRARY DESTINATION ${COLCON_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${COLCON_GLOBAL_LIB_DESTINATION}
  RUNTIME DESTINATION  ${COLCON_GLOBAL_LIB_DESTINATION}
  INCLUDES DESTINATION include
)

install(TARGETS ackerman_regulator_node
DESTINATION lib/${PROJECT_NAME}/)

ament_export_targets(ackerman_regulator_node HAS_LIBRARY_TARGET)


ament_package()
