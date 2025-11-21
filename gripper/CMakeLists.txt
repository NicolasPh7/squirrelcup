cmake_minimum_required(VERSION 3.8)
project(gripper)

# find_package(catkin REQUIRED)
find_package(ament_cmake REQUIRED)

# catkin_package()

# find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
endforeach(dir)

ament_package()
