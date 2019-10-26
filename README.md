# ROS Publisher Subscriber
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
This project follows the tutorial of writing a publisher and subscriber from
the  [ROS Wiki Page](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).
The project creates a catkin package, builds two nodes, name talker and listener, along with
making the CMakeList.txt and package.xml to incorporate the dependenies. Further, ROS service
is used to change the message being published to a user custom message.

---
## Dependencies
- The project uses Ubuntu 16.04
- The project uses Kinetic version of ROS. To install, follow the [link]( http://wiki.ros.org/kinetic/Installation/Ubuntu)
- The project uses catkin build system, To install, follow the [link](http://wiki.ros.org/catkin)

---
## Building the project
1. Create a catkin workspace \
`mkdir -p ~/catkin_ws/src` \
`cd ~/catkin_ws/` \
`catkin_make` \
2. Source the new setup files \
`source devel/setup.bash` \
3. Clone the repository\
`cd src/` \
`git clone https://github.com/Ytodi31/beginner_tutorials.git`\
4. Build the project \
`cd ..` \
`catkin_make`

---
## Running the project
Three terminals are required to run the nodes and one terminal is required to run
ROS Service in order to use custom string message
### Running the nodes
- Terminal 1 - ROS master \
`source ~/catkin_ws/devel/setup.bash` \
`roscore`
- Terminal 2 - Publisher node \
`source ~/catkin_ws/devel/setup.bash` \
`rosrun beginner_tutorials talker `
- Terminal 3 - Subscriber node\
`source ~/catkin_ws/devel/setup.bash` \
`rosrun beginner_tutorials listener `
### Running ROS Service
- Terminal 4 - ROS Service\
`source ~/catkin_ws/devel/setup.bash` \
`rosservice call /changeString " <Input desired custom string> "`
