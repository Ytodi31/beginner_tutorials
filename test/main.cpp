/**
 * @file main.cpp
 * @brief Main file for test files
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

 /**
  * @brief Main function for all test functions
  * @param Parameter 1, Number of inputs
  * @param Parameter 2, Input
  * @return boolean value
  */
 int main(int argc, char** argv) {
   ros::init(argc, argv,  "testBeginnerTutorials");
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
 }
