#!/bin/sh

rosrun graph_processing g2o_pcd_converter ~/.ros/g2o_output.g2o
rosrun graph_processing icp g2o_pointcloud.pcd test_data/test1.pcd 
pcd_viewer test_data/test1.pcd icp_output.pcd
