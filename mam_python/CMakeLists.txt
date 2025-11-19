cmake_minimum_required(VERSION 3.8)
project(mam_python)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS ${PROJECT_NAME}/ros2_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
