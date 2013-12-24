#!/usr/bin/env python

import sys
import roslib; roslib.load_manifest('bag_scripts')
import rospy
import rosbag

if len(sys.argv) < 2 :
  print "please provide 2 arguments: bagFileFixer <input.bag> <output.bag>"
  sys.exit()

input_file = str(sys.argv[1])
output_file = str(sys.argv[2])

i = 0
with rosbag.Bag(output_file, 'w') as output:
  for topic, msg, t in rosbag.Bag(input_file).read_messages():
    try:
      i += 1
      if not i % 100:
        rospy.loginfo(i)
      output.write(topic, msg, t)
    except:
      print "Unexpected error:", sys.exc_info()[0]
