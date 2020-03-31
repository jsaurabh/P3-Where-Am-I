cmake_minimum_required(VERSION 2.8.3)
project(ball_chaser)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
)

add_compile_options(-std=c++11)

add_service_files(
  FILES
  DriveToTarget.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(drive_bot src/drive_bot.cpp)
target_link_libraries(drive_bot ${catkin_LIBRARIES})
add_dependencies(drive_bot ball_chaser_generate_messages_cpp)

