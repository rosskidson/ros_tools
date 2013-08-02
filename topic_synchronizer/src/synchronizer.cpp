#include "ros/ros.h"
#include "std_msgs/String.h"

std::string topic_1;
std::string topic_2;

std::vector<ros::Time> callback_list_1;
std::vector<ros::Time> callback_list_2;

const static int queue_size = 1000;
const static uint average_window_size = 10;



void callback1(const std_msgs::String::ConstPtr& msg)
{
  callback_list_1.push_back(ros::Time::now());
}

void callback2(const std_msgs::String::ConstPtr& msg)
{
  callback_list_2.push_back(ros::Time::now());
}

// calculates time difference between callbacks based on callback_list_1 and callback_list_2
// iterates through list 1 and for each timestamp finds closest timestamp in list_2
// averages and returns timestamp difference
ros::Duration calcTopicTimeDiff()
{
  std::vector<ros::Duration> difference_list;
  typedef std::vector<ros::Time>::const_iterator TimeIterator;
  typedef std::vector<ros::Duration>::const_iterator DurationIterator;
  for(TimeIterator itr_outer=callback_list_1.begin(); itr_outer!=callback_list_1.end(); itr_outer++)
  {
    ros::Duration min_duration = ros::Duration(1000, 0);
    for(TimeIterator itr_inner=callback_list_2.begin(); itr_inner!=callback_list_2.end(); itr_inner++)
    {
      ros::Duration d = *itr_inner - *itr_outer;
      if(d.toNSec() < 0)
        d = *itr_outer - *itr_inner;
      if(d < min_duration)
        min_duration = d;
    }
    difference_list.push_back(min_duration);
  }
  ros::Duration total_duration;
  for(DurationIterator itr=difference_list.begin(); itr!=difference_list.end(); itr++)
    total_duration += *itr;
  double ave_seconds = total_duration.toSec()/difference_list.size();

  callback_list_1.clear();
  callback_list_2.clear();

  return ros::Duration(ave_seconds);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  if(argc < 2)
  {
    ROS_ERROR("please provide 2 arguments: <topic_name_1> <topic_name_2>");
    exit(0);
  }
  topic_1 = argv[1];
  topic_2 = argv[2];
  ros::NodeHandle nh;

  ros::Subscriber sub_1 = nh.subscribe(topic_1, queue_size, callback1);
  ros::Subscriber sub_2 = nh.subscribe(topic_2, queue_size, callback2);

  while(ros::ok())
  {
    if(callback_list_1.size() >= average_window_size &&
        callback_list_2.size() >= average_window_size)
      ROS_INFO_STREAM("difference between topics " << calcTopicTimeDiff());
    ros::spinOnce();
  }

  return 0;
}
