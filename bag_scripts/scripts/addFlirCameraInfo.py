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
    if topic == "/driving/flir/camera_info":
      msg.height = 480
      msg.width = 720
      msg.distortion_model = 'plumb_bob'
      msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
      msg.K = [1125, 0, 360, 0, 1000, 240, 0, 0, 1]
      msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
      msg.P = [1125, 0, 360, 0, 0, 1000, 240, 0, 0, 0, 0, 1]
      msg.binning_x = 0
      msg.binning_y = 0
      output.write(topic, msg, t)
    elif topic == "/driving/flir/image_raw/compressed":
      output.write(topic, msg, t)
