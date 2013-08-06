#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>

int
main (int argc, char** argv)
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f trafo;
	reader.read (argv[1], *cloudIn);
    std::ifstream myfile;
    myfile.open(argv[2]);
	myfile >> trafo(0,0);
	myfile >> trafo(0,1);
	myfile >> trafo(0,2);
	myfile >> trafo(0,3);
	myfile >> trafo(1,0);
	myfile >> trafo(1,1);
	myfile >> trafo(1,2);
	myfile >> trafo(1,3);
	myfile >> trafo(2,0);
	myfile >> trafo(2,1);
	myfile >> trafo(2,2);
	myfile >> trafo(2,3);
	myfile >> trafo(3,0);
	myfile >> trafo(3,1);
	myfile >> trafo(3,2);
	myfile >> trafo(3,3);

	transformPointCloud (*cloudIn, *cloudOut,  trafo);

	pcl::PCDWriter writer;
	writer.write ("output.pcd", *cloudOut, false);
  return (0);
}
