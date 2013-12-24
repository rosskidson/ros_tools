#!/usr/bin/env python


import sys
import roslib; roslib.load_manifest('bag_scripts')
import rospy
import rosbag

if len(sys.argv) < 2 :
  print "please provide 2 arguments: removeBadCamInfo <input.bag> <output.bag>"
  sys.exit()

input_file = str(sys.argv[1])
output_file = str(sys.argv[2])

with rosbag.Bag(output_file, 'w') as output:
  for topic, msg, t in rosbag.Bag(input_file).read_messages():
    if topic == "/driving/svc1/left/camera_info" or topic == "/driving/svc1/right/camera_info":
      if not msg.D[0] == 0.0:
        output.write(topic, msg, t)
    else:
      output.write(topic, msg, t)
