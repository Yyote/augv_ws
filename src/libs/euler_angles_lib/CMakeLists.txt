cmake_minimum_required(VERSION 3.8)
project(euler_angles_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(deps ament_cmake tf2 geometry_msgs tf2_geometry_msgs)

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


set(lib_sources include/euler_angles_lib/euler_angles.hpp src/euler_angles.cpp)

add_library(euler_angles ${lib_sources})
ament_target_dependencies(euler_angles ${deps}) # Добавление зависимостей


include_directories(include)

ament_export_targets(euler_angles HAS_LIBRARY_TARGET)

install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
)

install(
    TARGETS euler_angles
    EXPORT euler_angles
    LIBRARY DESTINATION ${COLCON_GLOBAL_LIB_DESTINATION}
    ARCHIVE DESTINATION ${COLCON_GLOBAL_LIB_DESTINATION}
    RUNTIME DESTINATION  ${COLCON_GLOBAL_BIN_DESTINATION}
    INCLUDES DESTINATION include
)




ament_package()
