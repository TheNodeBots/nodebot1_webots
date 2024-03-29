cmake_minimum_required(VERSION 3.8)
project(nodebot1_webots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)

include_directories(include)

add_executable(webots_chassis src/webots_chassis.cpp)
target_include_directories(webots_chassis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(webots_chassis PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  webots_chassis
  "rclcpp"
  "geometry_msgs"
  "webots_ros2_driver"
)

install(TARGETS 
  webots_chassis
  DESTINATION lib/${PROJECT_NAME}
)

# install python modules
ament_python_install_package(${PROJECT_NAME})

# install python executables
install(PROGRAMS
  scripts/main_script.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files etc.
install(
  DIRECTORY launch protos resource worlds
  DESTINATION share/${PROJECT_NAME}
)

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
