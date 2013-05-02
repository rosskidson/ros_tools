/*
 * main.cpp
 *
 *  Created on: Apr 30, 2013
 *      Author: Ross Kidson
 */


#include "feature_visualizer/feature_visualization.h"
#include <ros/ros.h>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "feature_visualizer");
  FeatureVisualization visualizer;
  ros::spin();
}
