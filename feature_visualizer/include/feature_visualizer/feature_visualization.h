/*
 * feature_visualization.h
 *
 *  Created on: Apr 30, 2013
 *      Author: ross kidson
 */

#ifndef FEATURE_PUBLISHER_H_
#define FEATURE_PUBLISHER_H_

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <feature_msgs/stereo_matches.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "image_buffer.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo> imageSyncPolicy;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo,
                                                        sensor_msgs::CameraInfo> stereoSyncPolicy;

typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::DisparityImage,
                                                        sensor_msgs::CameraInfo> disparitySyncPolicy;

class FeatureVisualization
{

typedef std::pair<sensor_msgs::Image, sensor_msgs::CameraInfo> imageWithInfo;
typedef std::pair<stereo_msgs::DisparityImage, sensor_msgs::CameraInfo> disparityWithInfo;

public:
  FeatureVisualization();

  ~FeatureVisualization(){};

  void stereoCallback(const sensor_msgs::ImageConstPtr& l_image_msg,
                      const sensor_msgs::ImageConstPtr& r_image_msg,
                      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                      const sensor_msgs::CameraInfoConstPtr& r_info_msg);

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                     const sensor_msgs::CameraInfoConstPtr& info_msg);

  void disparityCallback(const stereo_msgs::DisparityImageConstPtr& disparity_msg,
                         const sensor_msgs::CameraInfoConstPtr& info_msg);

  void featuresCallback(const feature_msgs::stereo_matchesConstPtr feature_locations_msg_);

  int findImageFromCamInfo(const sensor_msgs::CameraInfo& cam_info, std::vector<imageWithInfo>* target_vector);

  int findImageFromCamInfo(const sensor_msgs::CameraInfo& cam_info, std::vector<disparityWithInfo>* target_vector);

  void drawFeatures(const cv::Mat& image,
                     const std::vector<feature_msgs::coords>& current_features,
                     const std::vector<feature_msgs::coords>& previous_features,
                     cv::Mat& output_img);

private:
  ros::NodeHandle nh_;
  ros::Publisher left_image_publisher_;
  ros::Publisher right_image_publisher_;
  ros::Publisher dispaity_image_publisher_;

  ros::Subscriber image_subscriber_;
  ros::Subscriber stereo_subscriber_;
  ros::Subscriber features_sub_;

  message_filters::Synchronizer<imageSyncPolicy> *image_sync_;
  message_filters::Synchronizer<stereoSyncPolicy> *stereo_sync_;
  message_filters::Synchronizer<disparitySyncPolicy> *disparity_sync_;
  message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *image_cinfo_sub_;
  message_filters::Subscriber<sensor_msgs::Image> *left_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *left_image_cinfo_sub_;
  message_filters::Subscriber<sensor_msgs::Image> *right_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *right_image_cinfo_sub_;

  message_filters::Subscriber<stereo_msgs::DisparityImage> *disparity_image_sub_;

  std::vector<imageWithInfo> image_buffer_;
  std::vector<imageWithInfo> left_image_buffer_;
  std::vector<imageWithInfo> right_image_buffer_;
  std::vector<disparityWithInfo> disparity_image_buffer_;

  ImageBuffer<sensor_msgs::Image> mono_image_buffer_, left_iimage_buffer_, right_iimage_buffer_;


};

#endif // FEATURE_PUBLISHER_H_
