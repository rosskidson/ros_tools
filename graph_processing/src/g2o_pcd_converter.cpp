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
    std::cout << "please provide g2o filename to be converted\n";
    exit(0);
  }
  std::string filename(argv[1]);
  pcl::PointCloud<pcl::PointXYZ> cloud;

  std::vector<std::string> words;
  const std::string vertex_name = "G2O_VERTEX_SE3";
  std::ifstream in(filename.c_str());
  while(in) {
    std::string word;
    in >> word;
    if(word.compare(vertex_name) == 0)
    {
      in >> word;  //get rid of the id
      Eigen::Vector3d vec;
      for(int i=0; i<3; i++) in >> vec[i]; // read in x y z
      double q_in [4];
      for(int i=0; i<4; i++) in >> q_in[i];   // read in quaternion
      Eigen::Quaterniond q(q_in[3], q_in[0], q_in[1], q_in[2]);
      Eigen::Matrix3d rot_mat;
      rot_mat = q;                          //convert quat to 3x3 rotation mat
      Eigen::Matrix4d mat, mat_inv;
      mat.block<3,3>(0,0) = rot_mat;       //make 4x4 mat with rot and translation
      mat.block<3,1>(0,3) = vec;
      mat(3,3) = 1.0;
      mat_inv = mat.inverse();            // invert to get read 3d coordinates
      pcl::PointXYZ p;                    //create pcl point
      p.x = mat_inv(0,3);
      p.y = mat_inv(1,3);
      p.z = mat_inv(2,3);
      cloud.push_back(p);                // add to pointcloud
    }
  }

  pcl::io::savePCDFileASCII ("g2o_pointcloud.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to g2o_pointcloud.pcd" << std::endl;

//  for (size_t i = 0; i < cloud.points.size (); ++i)
//    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
