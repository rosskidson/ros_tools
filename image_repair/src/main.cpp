/*
 * main.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: Ross Kidson
 */
#include "image_io.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
  std::string input_topic = argv[1];
  std::string output_topic = argv[2];
  ros::init(argc, argv, "image_repair", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  ImageIO image_handle(nh, input_topic, output_topic);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
