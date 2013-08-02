/*
 * image_filtering.h
 *
 *  Created on: Jun 13, 2012
 *      Author: Ross Kidson
 */

#ifndef IMAGE_FILTERING_H
#define IMAGE_FILTERING_H

#include <ros/ros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/image_filter/ImageFilterConfig.h"

#include <opencv2/core/core.hpp>

class ImageFiltering
{
  public:
    ImageFiltering();
    ~ImageFiltering(){}
    void filterImage(cv::Mat& input_img, cv::Mat& output_img);

    bool isFrameDamaged(cv::Mat& input_img, cv::Mat &debug_img);

    void repairLines(cv::Mat &current_frame, cv::Mat &output_img);

  private:
    void reconfigCallback (image_filter::ImageFilterConfig &config, uint32_t level);

    bool checkRegionForCorruption(cv::Mat input);

    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<image_filter::ImageFilterConfig> reconfig_srv_;
    dynamic_reconfigure::Server<image_filter::ImageFilterConfig>::CallbackType reconfig_callback_;


    //paramters
    int median_filt_size_;
    int comparison_threshold_;
    double percentage_corruption_;
    double bilat_sigma_color_, bilat_sigma_space_;
    int bilat_d_, gauss_d_;

    int line_difference_threshold_;

    cv::Mat prev_frame_;

};


#endif // IMAGE_FILTERING_H_
