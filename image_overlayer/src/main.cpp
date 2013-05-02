/*
 * main.cpp
 *
 *  Created on: Apr 26, 2013
 *      Author: Ross Kidson
 */

#include <ros/ros.h>
#include "util.cpp"

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        stereo_msgs::DisparityImage
                                                        > syncPolicy;
const static int queue_size = 10;
const double alpha_ = 1.0;  //transparency for image
const double beta_ = 0.5;   //transparency for overlay
ros::NodeHandle *nh_;
ros::Subscriber subscriber_;
ros::Publisher publisher_;
message_filters::Synchronizer<syncPolicy>* synchronizer_ptr_;
message_filters::Subscriber<sensor_msgs::Image> *rgb_image_sub_;
message_filters::Subscriber<stereo_msgs::DisparityImage> *disparity_image_sub_;


void overlayImages(cv::Mat& image, cv::Mat& disparity, const float disparity_max, cv::Mat& output_img)
{
  // convert image to 3 channels so it may be overlayed with color
  cv::Mat image_rgb;
  if(image.channels() == 1)
    cv::cvtColor(image,image_rgb,CV_GRAY2RGB);
  else
    image_rgb = image;

  // use hsv color space to assign disparity to a color
  // hue = disparity
  // sat = 1
  // val = 0 (making the overall color black) where there is no disparity, otherwise 1
  cv::Mat disparity_img_hsv(disparity.rows, disparity.cols, CV_8UC3);
  for(int i=0; i< disparity.rows; i++) for(int j=0; j< disparity.cols; j++)
    disparity_img_hsv.at<cv::Vec3b>(i,j) =
      cv::Vec3b((disparity.at<float>(i,j)/disparity_max)*255, 255, 255*((disparity.at<float>(i,j) == -1) ? 0 : 1));

  //convert back to rgb for overlaying images
  cv::Mat disparity_img_rgb;
  cv::cvtColor(disparity_img_hsv,disparity_img_rgb, CV_HSV2RGB);

  cv::addWeighted( image_rgb, alpha_, disparity_img_rgb, beta_, 0.0, output_img);
}

void imgCallback (const sensor_msgs::ImageConstPtr& rgb_image_ptr,
                  const stereo_msgs::DisparityImageConstPtr& disparity_image_ptr)
{
  const float d_max = disparity_image_ptr->max_disparity - disparity_image_ptr->min_disparity;
  //convert sensor_msgs to cv mat
  cv::Mat image = convertSensorMsgToCV(rgb_image_ptr);
  sensor_msgs::ImagePtr img_ptr(new sensor_msgs::Image(disparity_image_ptr->image));
  cv::Mat disparity = convertSensorMsgToCV(img_ptr);
  cv::Mat output_img;

  overlayImages(image, disparity, d_max, output_img);
  publisher_.publish(convertCVToSensorMsg(output_img));
}

int main( int argc, char** argv )
{
  ros::init (argc, argv, "image_overlayer");
  nh_ = new ros::NodeHandle("~");
  publisher_ = nh_->advertise<sensor_msgs::Image> ("stereo_overlay", 1);
  rgb_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(*nh_, "/stereo/left/image_rect", queue_size);
  disparity_image_sub_ = new message_filters::Subscriber<stereo_msgs::DisparityImage>(*nh_, "/stereo/disparity", queue_size);
  synchronizer_ptr_ = new message_filters::Synchronizer<syncPolicy>(syncPolicy(queue_size),  *rgb_image_sub_, *disparity_image_sub_);
  synchronizer_ptr_->registerCallback(boost::bind(&imgCallback, _1, _2));
  ros::spin();
 return 0;
}
