/**
*BSD 3-Clause License
*
*Copyright (c) 2019, Yashaarth Todi
*All rights reserved.
*
*Redistribution and use in source and binary forms, with or without
*modification, are permitted provided that the following conditions are met:
*1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*3. Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
  auto client = nh.serviceClient<beginner_tutorials::customString>
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
