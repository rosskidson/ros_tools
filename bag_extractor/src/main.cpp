/*
 * main.cpp
 *
 *  Created on: Apr 26, 2013
 *      Author: Ross Kidson
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "util.cpp"

#include <sensor_msgs/Image.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

int main(int argc, char **argv) {
  std::stringstream bagname;
  bagname << "/data/2013-05-07-20-04-35.bag";

  rosbag::Bag bag;
  bag.open(bagname.str(), rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/stereo/left/image_rect"));
  topics.push_back(std::string("/stereo/right/image_rect"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ROS_INFO("opening bag time");
  int right_counter = 0;
  int left_counter = 0;
  char filename[50];
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    sensor_msgs::ImageConstPtr left_image_ptr, right_image_ptr;
    if(m.getTopic() == topics[0])
    {
      left_image_ptr = m.instantiate<sensor_msgs::Image>();
      cv::Mat left_image_mat = convertSensorMsgToCV(left_image_ptr);
      sprintf (filename, "rectified_Stereo__%06i-left.pnm", left_counter);
      cv::imwrite(filename, left_image_mat);
      left_counter++;
    }
    if(m.getTopic() == topics[1])
    {
      right_image_ptr = m.instantiate<sensor_msgs::Image>();
      cv::Mat right_image_mat = convertSensorMsgToCV(right_image_ptr);
      sprintf (filename, "rectified_Stereo__%06i-right.pnm", right_counter);
      cv::imwrite(filename, right_image_mat);
      right_counter++;
    }
    if(!( left_counter % 100))
      ROS_INFO_STREAM("message num " << left_counter);
  }
}
