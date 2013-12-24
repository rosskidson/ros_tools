#!/usr/bin/env python


import sys
import roslib; roslib.load_manifest('bag_scripts')
import rospy
import rosbag

if len(sys.argv) < 2 :
  print "please provide 2 arguments: fix_timestamps <input.bag> <output.bag>"
  sys.exit()

input_file = str(sys.argv[1])
output_file = str(sys.argv[2])

with rosbag.Bag(output_file, 'w') as output:
  for topic, msg, t in rosbag.Bag(input_file).read_messages():
    if (topic == "/driving/svc1/left/image_raw") or (topic == "/driving/svc1/left/camera_info") or (topic == "/driving/svc1/right/image_raw") or (topic == "/driving/svc1/right/camera_info"):
    # This also replaces tf timestamps under the assumption 
    # that all transforms in the message share the same timestamp
      output.write(topic, msg, msg.header.stamp if msg._has_header else t)
