cmake_minimum_required(VERSION 3.5)
project(rokae_description VERSION 0.0.4)
find_package(ament_cmake REQUIRED)

# Install folders: launch meshes urdf config
install(
  DIRECTORY launch meshes urdf config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executable scripts
install(PROGRAMS
  scripts/gui_to_joint_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

# This package is a pure description package (no code) and does not need to export dependencies.
ament_package()



#shangjialu 03/19