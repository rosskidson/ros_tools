#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//file io
#include <iomanip>
#include <iostream>
#include <fstream>

//constants
const static double preloop_wait = 0.5;
const static int loop_rate_hz = 1000;

int main(int argc, char** argv){
  if(argc < 3){
    ROS_ERROR("Please provide 3 arguments: rosrun trajectory_recorder trajectory_recorder <tf_parent> <tf_child> <output_filename>");
    exit(0);
  }
  static std::string tf_parent=argv[1];
  static std::string tf_child=argv[2];
  static std::string output_filename=argv[3];

  ros::init(argc, argv, "trajectory_recorder", ros::init_options::AnonymousName);
  ros::NodeHandle node;

  tf::TransformListener listener;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0) );

  ros::Time previous_stamp;

  std::ofstream file_output;
  file_output.open(output_filename.c_str());

  ROS_INFO_STREAM("Node ready to record tf " << tf_child);

  ros::Rate loop_rate(loop_rate_hz);
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try{
       listener.lookupTransform(tf_parent, tf_child, ros::Time(0), transform);

       if(previous_stamp != transform.stamp_)
       {
         file_output 
	  << std::setprecision(15) 
          << transform.stamp_ 
	  << " " << transform.getOrigin().x()
          << " " << transform.getOrigin().y()
          << " " << transform.getOrigin().z()
          << " " << transform.getRotation().x()
          << " " << transform.getRotation().y()
          << " " << transform.getRotation().z()
          << " " << transform.getRotation().w() << "\n";
         previous_stamp = transform.stamp_;
         ROS_INFO("recorded entry");
      }
    }
    catch (tf::TransformException ex){
       ROS_DEBUG("%s",ex.what());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  file_output.close();
  return 0;
};
