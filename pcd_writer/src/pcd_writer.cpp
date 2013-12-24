#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

int
  main (int argc, char** argv)
{
  if(argc < 2)
  {
    std::cout << "please provide filename to be converted. Format 'timestamp x y z'\n";
    exit(0);
  }
  std::string filename(argv[1]);
  pcl::PointCloud<pcl::PointXYZ> cloud;

  std::vector<std::string> words;
  const std::string key_name = "v";
  std::ifstream in(filename.c_str());
  while(in) {
    std::string word;
    in >> word;
    if(in.peek()  == '\n')
    {
      in >> word; //remove timestamp
      Eigen::Vector3d vec;
      for(int i=0; i<3; i++) in >> vec[i]; // read in x y z
      pcl::PointXYZ p;                    //create pcl point
      p.x = vec[0];
      p.y = vec[1];
      p.z = vec[2];
      cloud.push_back(p);                // add to pointcloud
    }
  }

  pcl::io::savePCDFileASCII ("pointcloud.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to pointcloud.pcd" << std::endl;

  return (0);
}
