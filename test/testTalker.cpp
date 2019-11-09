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
#include "tf/transform_listener.h"

/**
 * @brief This test function tests the service for changing the string
 */
TEST(PublisherTest, testCumstomStringService) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::customString>
  ("customString");

  beginner_tutorials::customString srv;
  srv.request.customStr = "This is a test string";
  // Calling service
  client.call(srv);

  // Test to check if string has changed
  EXPECT_EQ("This is a test string", srv.request.customStr);
}

/**
 * @brief This test function tests the broadcasted message of TF
 */
TEST(PublisherTest, testBroadcasterMessage) {
  // Creating objects
  tf::TransformListener listener;
  tf::StampedTransform transform;

  // Waiting for transform
  listener.waitForTransform("/world", "/talk", ros::Time(0),
                            ros::Duration(2.0));

  // Fetching broadcasted TF values
  listener.lookupTransform("/world", "/talk", ros::Time(0), transform);

  // Test to check if x and y coordinates are less than 1
  EXPECT_GE(1, transform.getOrigin().x());
  EXPECT_GE(1, transform.getOrigin().y());
}
