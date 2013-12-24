#!/usr/bin/env python


import sys
import roslib; roslib.load_manifest('bag_scripts')
import rospy
import rosbag

if len(sys.argv) < 2 :
  print "please provide 2 arguments: addCameraInfo <input.bag> <output.bag>"
  sys.exit()

input_file = str(sys.argv[1])
output_file = str(sys.argv[2])

i = 0
with rosbag.Bag(output_file, 'w') as output:
  for topic, msg, t in rosbag.Bag(input_file).read_messages():
    i += 1
    if not i % 100:
      rospy.loginfo(i)
    if "image_raw" in topic:
      msg.header.frame_id = "left_stereo_cam_estimate"
      output.write(topic, msg, t)
    elif "camera_info" in topic:
      msg.header.frame_id = "left_stereo_cam_estimate"
      output.write(topic, msg, t)
    else:
      output.write(topic, msg, t)
