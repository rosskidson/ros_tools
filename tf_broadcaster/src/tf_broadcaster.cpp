#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0) );

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
//    tf::StampedTransform transform;
//    try{
//       listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
//    }
//    catch (tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
//    }

    ROS_INFO("loop");
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/stereo_odom", "/kitty_stereo/left"));
    ros::spinOnce();
    ros::Duration(0.1).sleep();
//    loop_rate.sleep();
  }
  return 0;
};
