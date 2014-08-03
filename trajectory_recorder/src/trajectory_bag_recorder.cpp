#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <rosbag/bag.h>

//constants
const static double preloop_wait = 0.5;
const static int loop_rate_hz = 1000;

int main(int argc, char** argv){
  if(argc < 5){
    ROS_ERROR("Please provide 4 arguments: rosrun trajectory_recorder trajectory_recorder <tf_parent> <tf_child> <output_bagname> <topic_name>");
    exit(0);
  }
  static std::string tf_parent     =argv[1];
  static std::string tf_child      =argv[2];
  static std::string output_bagname=argv[3];
  static std::string topic_name    =argv[4];

  ros::init(argc, argv, "trajectory_recorder", ros::init_options::AnonymousName);
  ros::NodeHandle node;

  tf::TransformListener listener;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0) );

  ros::Time previous_stamp;

  rosbag::Bag bag;
  bag.open(output_bagname, rosbag::bagmode::Write);

  ROS_INFO_STREAM("Node ready to record tf " << tf_child);

  ros::Rate loop_rate(loop_rate_hz);
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(tf_parent, tf_child, ros::Time(0), transform);

      if(previous_stamp != transform.stamp_)
      {
        geometry_msgs::Pose pose;
        pose.orientation.x = transform.getRotation().x();
        pose.orientation.y = transform.getRotation().y();
        pose.orientation.z = transform.getRotation().z();
        pose.orientation.w = transform.getRotation().w();
        pose.position.x = transform.getOrigin().x();
        pose.position.y = transform.getOrigin().y();
        pose.position.z = transform.getOrigin().z();
        bag.write(topic_name, transform.stamp_, pose);
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
  bag.close();
  return 0;
};
