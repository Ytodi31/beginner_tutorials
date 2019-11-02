/**
 * @file talker.cpp
 * @brief Writing the Publisher Node
 *
 * This project contains the execution of tutorials from  ROS Wiki Page -
 * Writing a Simple Publisher and Subscriber
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Yashaarth Todi
 *
 * @date 10-26-2019
 */
#include <sstream>
#include <memory>
#include <string>
#include <cctype>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/customString.h"

// Defining a smart pointer to point to message to be published
std::unique_ptr<std::string> strP (new std::string);

/**
 * @brief Callback function that enables changing message of publisher
 * @param Parameter 1, request to service to change message
 * @param Parameter 2, response to service to change message
 * @return boolean value, true if message change sent
 */
bool changeString(beginner_tutorials::customString::Request &req,
  beginner_tutorials::customString::Response &res) {
    res.customStr = req.customStr;
    // Resetting the pointer to custom string given by user through service
    strP.reset(new std::string);
    *strP = res.customStr;
    ROS_DEBUG_STREAM_ONCE("Sending response to change string");
    return true;
  }

 /**
  * @brief This tutorial demonstrates simple sending of messages
  * over the ROS system with service to change publishing message
  * @param Parameter 1, Number of inputs
  * @param Parameter 2, Input
  * @return int, 0 if passed
  */
int main(int argc, char **argv) {
  std::string defaultString = "Default String";
  // Assigning the pointer to default string
  strP.reset(new std::string);
  *strP = defaultString;
  if (*strP == defaultString) {
    ROS_INFO_STREAM_ONCE("Default string can be changed using ROS Service");
  }

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * ServiceServer is an approach to call the services
   */
  ros::ServiceServer srvString = n.advertiseService("changeString",
  changeString);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  int frequency = 10;
  if (argc > 1) {
    std::string stringCheck = argv[1];
    // Checking for each character of parameter to be a numeric value
    for (auto& c : stringCheck)
    {
      if(isalpha(c))
      ROS_ERROR_STREAM_ONCE("Frequency should be a numeric value");
    }

    // Converting string to decimal value
    double tempFrequency = std::stod(argv[1]);
    frequency = std::stoi(argv[1]);
    if (tempFrequency - frequency !=0)
      ROS_WARN_STREAM("Floating value of frequency converted to integer");

    // Checks if frequency is positive
    if (frequency <= 0) {
      ROS_FATAL_STREAM("Frequency cannot be negative, try with positive value");
      return -1;
    }
  }
  ros::Rate loop_rate(frequency);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << *strP << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
