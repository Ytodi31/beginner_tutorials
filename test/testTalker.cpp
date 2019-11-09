/**
 * @file testTalker.cpp
 * @brief Test file for src/talker.cpp
 *
 * This project contains the execution of tutorials from  ROS Wiki Page -
 * Writing a Simple Publisher and Subscriber
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Yashaarth Todi
 *
 * @date 11-09-2019
 */

#include "ros/ros.h"
#include "gtest/gtest.h"
#include "beginner_tutorials/customString.h"

/**
 * @brief This test function tests the service for changing the string
 * @param none
 * @return none
 */
TEST(PublisherTest, testCumstomStringService) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::customString>
  ("customString");

  beginner_tutorials::customString srv;
  srv.request.customStr = "This is a test string";
  client.call(srv);
  EXPECT_EQ("This is a test string", srv.request.customStr)
}
