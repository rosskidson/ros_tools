/*
 * image_buffer.cpp
 *
 *  Created on: May 2, 2013
 *      Author: Ross Kidson
 */

#ifndef IMAGE_BUFFER_HPP_
#define IMAGE_BUFFER_HPP_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

template <class ImageType> class ImageBuffer
{
typedef std::pair<ImageType, sensor_msgs::CameraInfo> ImageWithInfo;

public:
    ImageBuffer(int buffer_size);

    void addImage(const ImageType& image, const sensor_msgs::CameraInfo& cinfo);

    ImageType* retrieveImageFromCamInfo(const sensor_msgs::CameraInfo& cinfo);

private:
  std::vector<ImageWithInfo> buffer_;
  uint buffer_size_;
};

template <class ImageType>
ImageBuffer<ImageType>::ImageBuffer(int buffer_size):
buffer_size_(buffer_size)
{
}

template <class ImageType>
void ImageBuffer<ImageType>::addImage(const ImageType& image, const sensor_msgs::CameraInfo& cinfo)
{
  if(buffer_.size() > buffer_size_)
    buffer_.erase(buffer_.begin());
  buffer_.push_back(ImageWithInfo(image, cinfo));
}

template <class ImageType>
ImageType* ImageBuffer<ImageType>::retrieveImageFromCamInfo(const sensor_msgs::CameraInfo& cinfo)
{
  for (int i = buffer_.size(); i >= 0; i--)
    if (buffer_[i].second.header.seq == cinfo.header.seq)
  {
      ROS_INFO_STREAM("idx " << i);
      return &(buffer_[i].first);
  }
  return NULL;
}


#endif // IMAGE_BUFFER_HPP_
