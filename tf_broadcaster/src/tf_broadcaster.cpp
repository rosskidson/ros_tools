#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  tf::Transform transform_1, transform_2, transform_3, transform_4;
  transform_1.setOrigin( tf::Vector3(576444.688813, 4139613.673761, 27.581482) );
  transform_1.setRotation( tf::Quaternion(-0.009, 0.010033, 0.967169, -0.253778) );

  transform_2.setOrigin(tf::Vector3(1.31747, 0.10535, 1.14755));
  transform_2.setRotation(tf::Quaternion(0.48207, -0.49406, 0.51857, -0.50458));

  transform_3.setOrigin( tf::Vector3(57.94599, -21.6628, -32.042));
  transform_3.setRotation( tf::Quaternion(-0.0071, -0.0202, -0.4469, 0.8943));

  transform_4.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
  transform_4.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0));

  ros::Rate loop_rate(1000);
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
    br.sendTransform(tf::StampedTransform(transform_1, ros::Time::now(), "/map", "/odometry_corrected_origin"));
    br.sendTransform(tf::StampedTransform(transform_2, ros::Time::now(), "/visual_odom_corrected_origin", "/left_stereo_cam_corrected"));

    br.sendTransform(tf::StampedTransform(transform_3, ros::Time::now(), "odom", "visual_odom_origin"));
    br.sendTransform(tf::StampedTransform(transform_2, ros::Time::now(), "visual_odom_origin", "left_stereo_cam"));

    br.sendTransform(tf::StampedTransform(transform_4, ros::Time::now(), "viso2_estimate", "camera"));

    ros::spinOnce();
//    ros::Duration(0.001).sleep();
    loop_rate.sleep();
  }
  return 0;
};
