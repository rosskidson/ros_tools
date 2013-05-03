/*
 * feature_visualization.cpp
 *
 *  Created on: Apr 30, 2013
 *      Author: ross kidson
 */


#include "feature_visualizer/feature_visualization.h"
#include "util.cpp"

#include <message_filters/subscriber.h>
#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>

const static std::string image_topic             = "/camera/mono/image_rect";
const static std::string image_cinfo_topic       = "/camera/mono/camera_info";
const static std::string left_image_topic        = "/stereo/left/image_rect";
const static std::string left_image_cinfo_topic  = "/stereo/left/camera_info";
const static std::string right_image_topic       = "/stereo/right/image_rect";
const static std::string right_image_cinfo_topic = "/stereo/right/camera_info";
const static std::string disparity_topic         = "/stereo/disparity";
const static std::string features_topic          = "/stereo_odometer/features";

const static std::string left_publish_topic      = "left/feature/overlay";
const static std::string right_publish_topic     = "right/feature/overlay";
const static std::string disparity_publish_topic = "used_disparity";

const static int queue_size = 10;

const static uint buffer_size = 10;
const static uint disparity_buffer_size = 10;

FeatureVisualization::FeatureVisualization():
    nh_ ("~"),
    mono_image_buffer_(buffer_size),
    left_image_buffer_(buffer_size),
    right_image_buffer_(buffer_size),
    disparity_buffer_(disparity_buffer_size)
{
    left_image_publisher_ = nh_.advertise<sensor_msgs::Image> (left_publish_topic, 1);
    right_image_publisher_ = nh_.advertise<sensor_msgs::Image> (right_publish_topic, 1);
    dispaity_image_publisher_ = nh_.advertise<sensor_msgs::Image> (disparity_publish_topic, 1);

    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, image_topic, queue_size);
    image_cinfo_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, image_cinfo_topic, queue_size);
    left_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, left_image_topic, queue_size);
    left_image_cinfo_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, left_image_cinfo_topic, queue_size);
    right_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, right_image_topic, queue_size);
    right_image_cinfo_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, right_image_cinfo_topic, queue_size);
    disparity_image_sub_ = new message_filters::Subscriber<stereo_msgs::DisparityImage>(nh_, disparity_topic, queue_size);

    image_sync_ = new message_filters::Synchronizer<imageSyncPolicy>(imageSyncPolicy(queue_size), *image_sub_, *image_cinfo_sub_);
    stereo_sync_ = new message_filters::Synchronizer<stereoSyncPolicy>(stereoSyncPolicy(queue_size), *right_image_sub_,
                                                                                                          *left_image_sub_,
                                                                                                          *right_image_cinfo_sub_,
                                                                                                          *left_image_cinfo_sub_);

    disparity_sync_ = new message_filters::Synchronizer<disparitySyncPolicy>(disparitySyncPolicy(queue_size), *disparity_image_sub_, *left_image_cinfo_sub_);

    features_sub_ = nh_.subscribe(features_topic, queue_size, &FeatureVisualization::featuresCallback, this);

    image_sync_->registerCallback(boost::bind(&FeatureVisualization::imageCallback, this, _1, _2));
    stereo_sync_->registerCallback(boost::bind(&FeatureVisualization::stereoCallback, this, _1, _2, _3, _4));
    disparity_sync_->registerCallback(boost::bind(&FeatureVisualization::disparityCallback, this, _1, _2 ));

}

void FeatureVisualization::stereoCallback(const sensor_msgs::ImageConstPtr& l_image_msg,
                                          const sensor_msgs::ImageConstPtr& r_image_msg,
                                          const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                          const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
  left_image_buffer_.addImage(*l_image_msg, *l_info_msg);
  right_image_buffer_.addImage(*r_image_msg, *r_info_msg);
}

void FeatureVisualization::imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                                         const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  mono_image_buffer_.addImage(*image_msg, *info_msg);
}

void FeatureVisualization::disparityCallback(const stereo_msgs::DisparityImageConstPtr& disparity_image_msg,
                                             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  disparity_buffer_.addImage(*disparity_image_msg, *info_msg);
}

void FeatureVisualization::featuresCallback(const feature_msgs::stereo_matchesConstPtr feature_locations_msg_)
{
  cv::Mat left_img, right_img, left_output_img, right_output_img;
  sensor_msgs::Image* img_ptr = left_image_buffer_.retrieveImageFromCamInfo(feature_locations_msg_->l_camera_info);
  if(img_ptr != NULL)
  {
    left_img = convertSensorMsgToCV(*img_ptr);
    drawFeatures(left_img, feature_locations_msg_->lc, feature_locations_msg_->lp, left_output_img);
    left_image_publisher_.publish(convertCVToSensorMsg(left_output_img));
  }
  stereo_msgs::DisparityImage* disp_ptr = disparity_buffer_.retrieveImageFromCamInfo(feature_locations_msg_->l_camera_info);
  // calculation of disparity sometimes fails
  // therefore just take the most recent one in the buffer when none is found for visualization
  if(disp_ptr == NULL)
    disp_ptr = disparity_buffer_.getLastImage();
  if((disp_ptr != NULL) && (img_ptr != NULL))
    overlayAndPublishDisparity(*disp_ptr, left_output_img);
}

void FeatureVisualization::drawFeatures(const cv::Mat& image,
                                         const std::vector<feature_msgs::coords>& current_features,
                                         const std::vector<feature_msgs::coords>& previous_features,
                                         cv::Mat& output_img)
{
  cv::Mat image_bw = image.clone();
  cvtColor(image_bw, output_img, CV_GRAY2RGB);
  for(uint i = 0; i < current_features.size(); i++)
  {
    cv::line(output_img, cv::Point(current_features[i].u, current_features[i].v),  cv::Point(previous_features[i].u, previous_features[i].v), cv::Scalar(100,255,255));
    cv::circle(output_img,  cv::Point(current_features[i].u, current_features[i].v), 1, cv::Scalar(0,255,255), /*thickness*/1, /*lineType*/8, /*shift*/0);
  }
}

void FeatureVisualization::overlayAndPublishDisparity(const stereo_msgs::DisparityImage& disparity_img, cv::Mat imageWithFeatures)
{
  //TODO:: find a better way to convert disparity img to cv.
  const float d_max = disparity_img.max_disparity - disparity_img.min_disparity;
  sensor_msgs::ImagePtr img_ptr(new sensor_msgs::Image(disparity_img.image));
  cv::Mat disparity = convertSensorMsgToCV(img_ptr);
  cv::Mat output_img;
  overlayImages(imageWithFeatures, disparity, d_max, output_img);
  dispaity_image_publisher_.publish(convertCVToSensorMsg(output_img));
}

void FeatureVisualization::overlayImages(cv::Mat& image, cv::Mat& disparity, const float disparity_max, cv::Mat& output_img)
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

  cv::addWeighted( image_rgb, 1.0, disparity_img_rgb, 0.5, 0.0, output_img);
}


