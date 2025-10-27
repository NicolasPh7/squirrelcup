cmake_minimum_required(VERSION 3.8)
project(mam_eurobot_2026)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)


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

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${pcl_conversions_INCLUDE_DIRS})

# Create and install Cpp executable
add_executable(cpp_test src/cpp_test.cpp)
ament_target_dependencies(cpp_test rclcpp geometry_msgs)
install(TARGETS cpp_test DESTINATION lib/${PROJECT_NAME})

add_executable(nut_identifier src/nut_identifier.cpp)
target_link_libraries(nut_identifier ${OpenCV_LIBS} ${tf2_LIBRARIES})
ament_target_dependencies(
  nut_identifier
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  visualization_msgs
)
install(TARGETS nut_identifier DESTINATION lib/${PROJECT_NAME})

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs)
install(TARGETS teleop DESTINATION lib/${PROJECT_NAME})

add_executable(inertial_odometry src/inertial_odometry.cpp)
ament_target_dependencies(inertial_odometry rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(inertial_odometry ${tf2_LIBRARIES})
install(TARGETS inertial_odometry DESTINATION lib/${PROJECT_NAME})

add_executable(robot_controller src/robot_controller.cpp)
ament_target_dependencies(robot_controller rclcpp geometry_msgs nav_msgs visualization_msgs tf2 tf2_geometry_msgs cv_bridge OpenCV)
target_link_libraries(robot_controller ${tf2_LIBRARIES})
install(TARGETS robot_controller DESTINATION lib/${PROJECT_NAME})

# add_executable(trajectory_planner src/trajectory_planner.hpp)
# ament_target_dependencies(trajectory_planner rclcpp geometry_msgs nav_msgs visualization_msgs tf2 tf2_geometry_msgs)
# target_link_libraries(trajectory_planner ${tf2_LIBRARIES})
# install(TARGETS trajectory_planner DESTINATION lib/${PROJECT_NAME})

add_executable(object_detector src/object_detector.cpp)
ament_target_dependencies(object_detector rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_geometry_msgs cv_bridge OpenCV)
target_link_libraries(object_detector ${PCL_LIBRARIES})
install(TARGETS object_detector DESTINATION lib/${PROJECT_NAME})

add_executable(aruco_localization src/aruco_localization.cpp)
ament_target_dependencies(aruco_localization rclcpp sensor_msgs geometry_msgs visualization_msgs cv_bridge OpenCV tf2 tf2_geometry_msgs cv_bridge OpenCV)
install(TARGETS aruco_localization DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install worlds files
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# Install models files
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS ${PROJECT_NAME}/py_test.py DESTINATION lib/${PROJECT_NAME})

ament_package()

if(BUILD_TESTING)
  # Register pytest tests
  ament_add_pytest_test(test_integration test/test_integration_odometry.py)
  ament_add_pytest_test(test_odometry test/test_odometry.py)
endif()
