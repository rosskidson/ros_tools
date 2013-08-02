/*
 * image_io.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: Ross Kidson
 */

#include "image_io.h"
#include "util.cpp"
#include <opencv2/highgui/highgui.hpp>

//const static std::string input_topic = "/kitty_stereo/right/image_rect";
//const static std::string output_topic = "/stereo/right/image_rect";
const static std::string debug_topic = "/image_filter_debug";

ImageIO::ImageIO(ros::NodeHandle nh, std::string input_topic, std::string output_topic):
nh_(nh),
image_transport_(nh_),
subscriber_(image_transport_.subscribe(input_topic, 1, boost::bind(&ImageIO::imageCallback, this, _1))),
publisher_(image_transport_.advertise(output_topic, 1)),
publisher_debug_(image_transport_.advertise(debug_topic, 1))
{
}

void ImageIO::imageCallback(sensor_msgs::ImageConstPtr ros_image_ptr)
{
  cv::Mat input_img = convertSensorMsgToCV(ros_image_ptr);
  publisher_.publish(convertCVToSensorMsg(input_img));

}
