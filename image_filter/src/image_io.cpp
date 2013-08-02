/*
 * image_io.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: Ross Kidson
 */

#include "image_io.h"
#include "util.cpp"
#include "image_filtering.h"

const static std::string input_topic = "/image_original";
const static std::string output_topic = "/image_filtered";
const static std::string debug_topic = "/image_filter_debug";

ImageIO::ImageIO(ros::NodeHandle nh):
nh_(nh),
image_transport_(nh_),
subscriber_(image_transport_.subscribe(input_topic, 1, boost::bind(&ImageIO::imageCallback, this, _1))),
publisher_(image_transport_.advertise(output_topic, 1)),
publisher_debug_(image_transport_.advertise(debug_topic, 1)),
filter_()
{}

void ImageIO::imageCallback(sensor_msgs::ImageConstPtr ros_image_ptr)
{
  cv::Mat input_img = convertSensorMsgToCV(ros_image_ptr);
  cv::Mat output_img_1;
  cv::Mat output_img_2;
  //call filt
  bool dropFrame = (filter_.isFrameDamaged(input_img, output_img_1));
  publisher_debug_.publish(convertCVToSensorMsg(output_img_1));
  if(dropFrame)
    return;
  filter_.filterImage(input_img, output_img_2);
//  filter_.repairLines(input_img, output_img_2);
  publisher_.publish(convertCVToSensorMsg(output_img_2));

}
