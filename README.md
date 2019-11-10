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
- The project uses GTest framework for unit testing with ROS

---
## Building the project
1. Create a catkin workspace \
`mkdir -p ~/catkin_ws/src` (Skip this step if you have an exisitng catkin worksapce)\
`cd ~/catkin_ws/` \
`catkin_make`
2. Source the new setup files \
`source devel/setup.bash`
3. Clone the repository\
`cd src/` \
`git clone https://github.com/Ytodi31/beginner_tutorials.git`
4. Build the project \
`cd ..` \
`catkin_make`

---
## Running the project
One terminal is required to launch both nodes as described in Option 1 and one terminal is required to run ROS Service to use custom string message.
Three terminals are required to run the nodes as described in Option 2 and one terminal is required to run ROS Service in order to use custom string message.
One terminal will be required if TF frame messages to be viewed.

### 1. Using launch Files
**Changing Frequency of publishing messages :**
- With default frequency \
`source ~/catkin_ws/devel/setup.bash` \
`roslaunch beginner_tutorials beginnerTutorials.launch`

- With user defined frequency \
`source ~/catkin_ws/devel/setup.bash` \
`roslaunch beginner_tutorials beginnerTutorials.launch frequency:=1`\
 Instead of `1` in the above command, enter the desired frequency (positive value)

**Recording messages published on all topics in a bag file :**
- By default, on launching the project, the messages will be recorded and stored in a bag file
- To disable recording, launch the file with the following argument :\
`roslaunch beginner_tutorials beginnerTutorials.launch recordBag:=false`
- To diable recording and change frequency, run the following :\
`roslaunch beginner_tutorials beginnerTutorials.launch frequency:=1 recordBag:=false`

### 2. Running the nodes
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

### TF frames
To view TF frames between the "world" and "talk" frames, run the project following steps mentioned above and then execute the following in a new terminal :

- To view broadcasted messages:\
`source ./devel/setup.bash` \
`rosrun tf tf_echo /world /talk`

- To view the relation using RQT graph:\
`source ./devel/setup.bash` \
`rosrun rqt_tf_tree rqt_tf_tree`

- To generate a pdf of the the TF frames and view it:\
`source ./devel/setup.bash` \
`rosrun tf view_frames` \
`evince frames.pdf`\
A sample of the PDF can be viewed [here](https://github.com/Ytodi31/beginner_tutorials/blob/Week11_HW/Results/frames.pdf).

---
## Unit Testing - Rostest
To build and run the tests :\
`cd ~/catkin_ws/`\
`source ./devel/setup.bash` \
`catkin_make `\
`run_tests_beginner_tutorials` \
`rostest beginner_tutorials testBeginnerTutorials.test `

---
## Bag file
- To inspect the recorded bag file :\
`cd <path to directory>/Results`\
`rosbag info record.bag`

- To play the recorded bag file\
 `cd <path to directory>/Results`\
 `rosbag play record.bag`
- To play the recorded bag file with Listener node\
 Ensure that talker node is not running
 - Terminal 1 - ROS master \
 `source ~/catkin_ws/devel/setup.bash` \
 `roscore`

 - Terminal 2 - Subscriber node\
 `source ~/catkin_ws/devel/setup.bash` \
 `rosrun beginner_tutorials listener `

 - Terminal 3 - rosbag play\
 `cd <path to directory>/Results`\
 `rosbag play record.bag`

---

### Visualize in RQT
To viualize publisher subscriber relationship, run the following in a new terminal \
`source ~/catkin_ws/devel/setup.bash` \
`rosrun rqt_graph rqt_graph`
