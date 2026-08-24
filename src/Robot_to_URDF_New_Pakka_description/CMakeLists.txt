cmake_minimum_required(VERSION 3.8)
project(Robot_to_URDF_New_Pakka_description)

find_package(ament_cmake REQUIRED)

# Install package data
install(DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
