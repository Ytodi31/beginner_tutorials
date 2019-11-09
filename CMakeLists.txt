cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile with C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

## Declaring ROS Services
add_service_files(FILES customString.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

## Declaring a Catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(testBeginnerTutorials test/testBeginnerTutorials.test test/main.cpp test/testTalker.cpp)
  target_link_libraries(testBeginnerTutorials ${catkin_LIBRARIES})
endif()
