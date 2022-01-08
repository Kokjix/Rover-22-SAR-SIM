cmake_minimum_required(VERSION 2.8.3)
project(blender_gazebo)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB xacro_files worlds/*.world.xacro)
xacro_add_files(${xacro_files} TARGET generate_world_files)

install(DIRECTORY blender_gazebo worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/install
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.world"
)
