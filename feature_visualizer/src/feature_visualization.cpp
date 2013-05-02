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

const static uint stereo_buffer_size = 10;
const static uint image_buffer_size = 10;
const static uint disparity_buffer_size = 10;

FeatureVisualization::FeatureVisualization():
    nh_ ("~"),
    mono_image_buffer_(10),
    left_iimage_buffer_(10),
    right_iimage_buffer_(10)
{
    left_image_publisher_ = nh_.advertise<sensor_msgs::Image> (left_publish_topic, 1);
    right_image_publisher_ = nh_.advertise<sensor_msgs::Image> (right_publish_topic, 1);
    dispaity_image_publisher_ = nh_.advertise<stereo_msgs::DisparityImage> (disparity_publish_topic, 1);

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
  left_iimage_buffer_.addImage(*l_image_msg, *l_info_msg);
  right_iimage_buffer_.addImage(*r_image_msg, *r_info_msg);
  //if(left_image_buffer_.size() > stereo_buffer_size)
  //{
  //  left_image_buffer_.erase(left_image_buffer_.begin());
  //  right_image_buffer_.erase(right_image_buffer_.begin());
  //}
  //left_image_buffer_.push_back(imageWithInfo(*l_image_msg, *l_info_msg));
  //right_image_buffer_.push_back(imageWithInfo(*r_image_msg, *r_info_msg));
}

void FeatureVisualization::imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                                         const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  mono_image_buffer_.addImage(*image_msg, *info_msg);
}

void FeatureVisualization::disparityCallback(const stereo_msgs::DisparityImageConstPtr& disparity_image_msg,
                                             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if(disparity_image_buffer_.size() > disparity_buffer_size)
    disparity_image_buffer_.erase(disparity_image_buffer_.begin());
  disparity_image_buffer_.push_back(disparityWithInfo(*disparity_image_msg, *info_msg));
}

void FeatureVisualization::featuresCallback(const feature_msgs::stereo_matchesConstPtr feature_locations_msg_)
{
  cv::Mat left_img, right_img, left_output_img, right_output_img;
//  const int l_img_idx = findImageFromCamInfo(feature_locations_msg_->l_camera_info, &left_image_buffer_);
//  ROS_DEBUG_STREAM("left img idx: " << l_img_idx);
//  if (l_img_idx > -1)
//  {
//    left_img = convertSensorMsgToCV(left_image_buffer_[l_img_idx].first);
//    drawFeatures(left_img, feature_locations_msg_->lc, feature_locations_msg_->lp, left_output_img);
//  }
//
//  const int r_img_idx = findImageFromCamInfo(feature_locations_msg_->r_camera_info, &right_image_buffer_);
//  ROS_DEBUG_STREAM("right img idx: " << l_img_idx);
//  if (r_img_idx > -1)
//  {
//    right_img = convertSensorMsgToCV(right_image_buffer_[r_img_idx].first);
//    drawFeatures(right_img, feature_locations_msg_->rc, feature_locations_msg_->rp, right_output_img);
//  }
//
//  const int d_img_idx = findImageFromCamInfo(feature_locations_msg_->l_camera_info, &disparity_image_buffer_);
//  ROS_DEBUG_STREAM("disp img idx: " << d_img_idx);
//  if (d_img_idx > -1)
//    dispaity_image_publisher_.publish(disparity_image_buffer_[d_img_idx].first);
//
//  left_image_publisher_.publish(convertCVToSensorMsg(left_output_img));
//  right_image_publisher_.publish(convertCVToSensorMsg(right_output_img));

  sensor_msgs::Image* img_ptr = left_iimage_buffer_.retrieveImageFromCamInfo(feature_locations_msg_->l_camera_info);
  if(img_ptr != NULL)
  {
    left_img = convertSensorMsgToCV(*img_ptr);
    drawFeatures(left_img, feature_locations_msg_->lc, feature_locations_msg_->lp, left_output_img);
    left_image_publisher_.publish(convertCVToSensorMsg(left_output_img));
  }
}

int FeatureVisualization::findImageFromCamInfo(const sensor_msgs::CameraInfo& cam_info, std::vector<imageWithInfo>* target_vector)
{
  for (std::vector<imageWithInfo>::const_iterator itr = target_vector->end(); itr != target_vector->begin(); itr--)
    if (itr->second.header.seq == cam_info.header.seq)
      return (itr - target_vector->begin());
  return -1;
}

int FeatureVisualization::findImageFromCamInfo(const sensor_msgs::CameraInfo& cam_info, std::vector<disparityWithInfo>* target_vector)
{
  for (std::vector<disparityWithInfo>::const_iterator itr = target_vector->end(); itr != target_vector->begin(); itr--)
    if (itr->second.header.seq == cam_info.header.seq)
      return (itr - target_vector->begin());
  return -1;
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

   // ROS_INFO_STREAM("u " << current_features[i].u << " v " << current_features[i].v);
  }
}


