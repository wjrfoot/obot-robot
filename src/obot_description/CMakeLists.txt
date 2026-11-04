cmake_minimum_required(VERSION 3.8)
project(obot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tools
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
