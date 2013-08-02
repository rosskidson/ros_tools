/*
 * main.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: Ross Kidson
 */
#include "image_io.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_filter");
  ros::NodeHandle nh;
  ImageIO image_handle(nh);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
