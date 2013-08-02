/*
 * image_io.h
 *
 *  Created on: Jun 13, 2012
 *      Author: Ross Kidson
 */

#ifndef IMAGE_IO_H
#define IMAGE_IO_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>


class ImageIO
{
  public:
    ImageIO(ros::NodeHandle nh, std::string input_topic, std::string output_topic);
    ~ImageIO(){}

  private:
    void imageCallback(sensor_msgs::ImageConstPtr image_ptr);

    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher_, publisher_debug_;

};


#endif // IMAGE_IO_H
