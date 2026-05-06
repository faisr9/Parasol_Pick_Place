cmake_minimum_required(VERSION 3.5)
project(p3_cell_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
