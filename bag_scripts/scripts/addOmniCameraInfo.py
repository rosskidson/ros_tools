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
    if topic == "/stereo/omni/camera_info":
      msg.height = 960
      msg.width = 1280
      msg.distortion_model = 'CAMERA_MODEL_SCARAMUZZA'
      msg.D = [1280,960,659.5194366,491.8820383,227,496,-239.7784375,0,0.0008284612713,2.339443774e-06,-1.955686923e-09,-1,-0.1e-09,0,-1,25,3,0,0,0]
      msg.K = [0,0,0,0,0,0,0,0,0]
      msg.R = [0,0,0,0,0,0,0,0,0]
      msg.P = [0,0,0,0,0,0,0,0,0,0,0,0]
      msg.binning_x = 0
      msg.binning_y = 0
      output.write(topic, msg, t)
    else:
      output.write(topic, msg, t)
