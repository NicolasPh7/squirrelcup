cmake_minimum_required(VERSION 3.8)
project(mam_eurobot_2026)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nut_identifier_msgs REQUIRED)


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

# Include Cpp "include" directory
include_directories(include)

# Create and install Cpp executable
add_executable(cpp_test src/cpp_test.cpp)
ament_target_dependencies(cpp_test rclcpp geometry_msgs)
install(TARGETS cpp_test DESTINATION lib/${PROJECT_NAME})

add_executable(nut_identifier src/nut_identifier.cpp)
target_link_libraries(nut_identifier ${OpenCV_LIBS})
ament_target_dependencies(
  nut_identifier
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV)
install(TARGETS nut_identifier DESTINATION lib/${PROJECT_NAME})

add_executable(nut_identifier2 src/nut_identifier2.cpp)
ament_target_dependencies(nut_identifier2 rclcpp sensor_msgs geometry_msgs cv_bridge nut_identifier_msgs OpenCV)
install(TARGETS nut_identifier2 DESTINATION lib/${PROJECT_NAME})

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs)
install(TARGETS teleop DESTINATION lib/${PROJECT_NAME})

add_executable(inertial_odometry src/inertial_odometry.cpp)
ament_target_dependencies(inertial_odometry rclcpp geometry_msgs nav_msgs)
target_link_libraries(inertial_odometry ${tf2_LIBRARIES})
ament_target_dependencies(inertial_odometry tf2 tf2_ros)
install(TARGETS inertial_odometry DESTINATION lib/${PROJECT_NAME})


# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install worlds files
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# Install models files
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS ${PROJECT_NAME}/py_test.py DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()

if(BUILD_TESTING)
  # Register pytest tests
  ament_add_pytest_test(test_integration test/test_integration_odometry.py)
  ament_add_pytest_test(test_odometry test/test_odometry.py)
  ament_add_pytest_test(test_nut_identifier2 test/test_nut_identifier2.py)
endif()
