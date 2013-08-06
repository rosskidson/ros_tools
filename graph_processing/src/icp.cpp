#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{
  if(argc < 2){
    std::cout << "please provide 2 pcd files to be registered\n";
    exit(0);
  }
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read (argv[1], *cloud_source);
  reader.read (argv[2], *cloud_target);

  pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_source);
  icp.setInputTarget(cloud_target);
  icp.setMaxCorrespondenceDistance(100);
  icp.setRANSACOutlierRejectionThreshold(5.0);
  icp.setMaximumIterations(10000);


  icp.align(*cloud_out);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::PCDWriter writer;
  writer.write ("icp_output.pcd", *cloud_out, false);


 return (0);
}

