cmake_minimum_required(VERSION 2.8.3)
project(iiwa_tool_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_msgs
  geometry_msgs
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(read_robot_state src/read_robot_state.cpp)
target_link_libraries(read_robot_state ${catkin_LIBRARIES})

add_executable(command_robot src/command_robot.cpp)
target_link_libraries(command_robot ${catkin_LIBRARIES})

add_executable(command_robot_moveit src/command_robot_moveit.cpp)
target_link_libraries(command_robot_moveit ${catkin_LIBRARIES})