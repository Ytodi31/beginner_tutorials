cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declaring ROS Messages
# add_message_files(FILES Message1.msg Message2.msg)

## Declaring ROS Services
# add_service_files(FILES Service1.srv Service2.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

## Declaring a Catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})

## Specify dependencies to link a library or executable target against
add_dependencies(talker beginner_tutorials_generate_messages_cpp)
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
