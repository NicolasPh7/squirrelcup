cmake_minimum_required(VERSION 3.8)
project(robot_v3)

# find_package(catkin REQUIRED)
find_package(ament_cmake REQUIRED)

# catkin_package()

# find_package(roslaunch)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

foreach(dir config launch meshes urdf textures)
	install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
endforeach(dir)

ament_package()
