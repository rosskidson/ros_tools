/*
 * image_filtering.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: Ross Kidson
 */

#include "image_filtering.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

const static int lense_x1 = 300;
const static int lense_y1 = 160;
const static int lense_width = 260;
const static int lense_height = 255;

const static int corner1_x = 1;
const static int corner1_y = 422;
const static int corner1_width = 36;
const static int corner1_height = 77;

const static int corner2_x = 37;
const static int corner2_y = 480;
const static int corner2_width = 26;
const static int corner2_height = 86;

const static int corner3_x = 63;
const static int corner3_y = 546;
const static int corner3_width = 50;
const static int corner3_height = 53;

ImageFiltering::ImageFiltering():
nh_("~/filter_settings"),
reconfig_srv_(nh_),
median_filt_size_(9)
{
  reconfig_callback_ = boost::bind (&ImageFiltering::reconfigCallback, this, _1, _2);
  reconfig_srv_.setCallback (reconfig_callback_);
}

void ImageFiltering::reconfigCallback (image_filter::ImageFilterConfig &config,
    uint32_t level)
{
  comparison_threshold_=config.comparison_threshold;
  percentage_corruption_=config.percentage_corruption;
  median_filt_size_=config.median_filter_size;
  bilat_d_ = config.bilat_d;
  bilat_sigma_color_ = config.bilat_sigma_color;
  bilat_sigma_space_ = config.bilat_sigma_space;
  gauss_d_ = config.gauss_d;
  line_difference_threshold_ = config.line_difference_threshold;
}

// isFrameDamaged
//  This function is used to filter out frames that are too badly
//  corrupted to be used.  It does this by checking that the center
//  blind spot or the edges (which should be very dark) are in fact dark
bool ImageFiltering::isFrameDamaged(cv::Mat &input_img, cv::Mat& debug_img)
{
  debug_img = input_img;
  cv::Mat bw_input_img;
  if(input_img.channels() > 1)
    cv::cvtColor(input_img,bw_input_img,CV_RGB2GRAY);
  else
    bw_input_img = input_img;
  bool pass;
  pass = checkRegionForCorruption(bw_input_img(cv::Rect(lense_x1, lense_y1, lense_width, lense_height)));
  pass = pass || checkRegionForCorruption(bw_input_img(cv::Rect(corner1_x, corner1_y, corner1_width, corner1_height)));
  pass = pass || checkRegionForCorruption(bw_input_img(cv::Rect(corner2_x, corner2_y, corner2_width, corner2_height)));
  pass = pass || checkRegionForCorruption(bw_input_img(cv::Rect(corner3_x, corner3_y, corner3_width, corner3_height)));
  return pass;
}

// checkRegionForCorruption.  Returns true if the region is not black enough

bool ImageFiltering::checkRegionForCorruption(cv::Mat input)
{
  cv::Mat diff_threshold;
  cv::threshold(input, diff_threshold, comparison_threshold_, 255, 0);
  return (((cv::countNonZero(diff_threshold)*100.0)/(double)(lense_height*lense_width)) > percentage_corruption_);
}


void ImageFiltering::filterImage(cv::Mat &input_img, cv::Mat &output_img)
{
  cv::Mat median_blured, bilat_blured;
  cv::Mat median_blured1, median_blured2, median_blured3, median_blured4;
  cv::medianBlur(input_img, median_blured, median_filt_size_);
  cv::bilateralFilter(median_blured, bilat_blured, bilat_d_, bilat_sigma_color_, bilat_sigma_space_);
  cv::GaussianBlur(bilat_blured, output_img, cv::Size(gauss_d_,gauss_d_),0,0);
}

// the following is an attempt to indentify and replace broken lines
// it is too difficult to threshold the difference between noise and movement

void ImageFiltering::repairLines(cv::Mat& current_frame, cv::Mat& output_img)
{
  cv::Mat current_frame_original;
  if(prev_frame_.cols == 0)
  {
    prev_frame_ = current_frame.clone();
    return;
  }else
    current_frame_original = current_frame.clone();

//  The following replaces broken lines based on number of "different" pixels
//  for(int i=1; i<current_frame.rows; i++)
//  {
//    cv::Mat current_line(current_frame(cv::Rect(0,i,current_frame.cols,1)));
//    cv::Mat prev_line(prev_frame_(cv::Rect(0,i,current_frame.cols,1)));
//    cv::Mat diff, diff_thr;
//    cv::absdiff(current_line, prev_line, diff);
//    cv::threshold(diff, diff_thr, comparison_threshold_, 255, 0);
//    int total_diff = cv::countNonZero(diff_thr);
//    if(total_diff > line_difference_threshold_)
//    {
//      ROS_INFO_STREAM("line copied, diff " << total_diff);
//      cv::Mat (current_frame(cv::Rect(0,i-1,current_frame.cols,1))).copyTo(current_line);
//    }
//  }

// this is to show a thresholded difference image
  cv::Mat diff, diff_thr;
  cv::absdiff(current_frame, prev_frame_, diff);
  cv::threshold(diff, output_img, comparison_threshold_, 255, 0);
  prev_frame_ = current_frame_original;
}
