cmake_minimum_required(VERSION 3.8)
project(dexsent_description)
find_package(ament_cmake REQUIRED)
# Install the URDF folder so MoveIt can see it
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
ament_package()
