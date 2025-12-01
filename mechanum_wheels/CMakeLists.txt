cmake_minimum_required(VERSION 3.8)
project(mechanum_wheels)

find_package(ament_cmake REQUIRED)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

foreach(dir launch meshes urdf worlds)
	install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
endforeach(dir)

ament_package()
