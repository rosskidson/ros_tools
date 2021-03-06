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
    if topic == "/stereo/left/camera_info":
      msg.height = 480
      msg.width = 640
      msg.distortion_model = 'plumb_bob'
      msg.D = [-0.30156922368780997, 0.15953706855157301, 0.00191176766756842, 0.00092010310690901801, 0.0]
      msg.K = [666.49815633162098, 0.0, 326.102166453731, 0.0, 666.16935609307598, 219.00195737196, 0.0, 0.0, 1.0]
      msg.R = [0.99984116566852299, -0.0024479059949448299, -0.0176536452567073, 0.0025368376130038301, 0.99998419723473297, 0.0050169448603183597, 0.0176410852708953, -0.0050609324285292596, 0.99983157535327905]
      msg.P = [671.02816422909905, 0.0, 339.69039535522501, 0.0, 0.0, 671.02816422909905, 224.89588928222699, 0.0, 0.0, 0.0, 1.0, 0.0]
      msg.binning_x = 0
      msg.binning_y = 0
      output.write(topic, msg, t)
    elif topic == "/stereo/right/camera_info":
      msg.height = 480
      msg.width = 640
      msg.distortion_model = 'plumb_bob'
      msg.D = [-0.30416602667055798, 0.18496146890446699, -0.00095544294480704795, 0.00097864599100944897, 0.0]
      msg.K = [668.70073413878697, 0.0, 320.69862636797899, 0.0, 668.27209086554501, 231.711397674143, 0.0, 0.0, 1.0]
      msg.R = [0.99974315620264598, -0.0055942943245342203, -0.021961910139635599, 0.00548354706808346, 0.99997196157087498, -0.0050996846615643599, 0.021989823499133799, 0.0049779456712348603, 0.99974580155126003]
      msg.P = [671.02816422909905, 0.0, 339.69039535522501, -129.218187531583, 0.0, 671.02816422909905, 224.89588928222699, 0.0, 0.0, 0.0, 1.0, 0.0]
      msg.binning_x = 0
      msg.binning_y = 0
      output.write(topic, msg, t)
    else:
      output.write(topic, msg, t)
