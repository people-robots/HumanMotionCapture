#!/usr/bin/env python
from __future__ import print_function #needs to be the first line of code in your script to bring the "print" function from Python 3 into Python 2.6+.

import roslib
#roslib.load_manifest('HamandiM')
import sys
import rospy #rospy is a pure Python client library for ROS. The rospy client API enables Python programmers to quickly interface with ROS Topics, Services, and Parameters.
from std_msgs.msg import Bool #Standard ROS Messages. std_msgs contains wrappers for ROS primitive types. 

def main(args):
    rospy.init_node('button', anonymous=True) #initializes the ROS node for the process. You can only have one node in a rospy process.
    rate = rospy.Rate(1) #maintaining a particular rate (1hz) for a loop with rate.sleep()
    pub = rospy.Publisher('/button',Bool, queue_size=10); # rospy.Publisher(topic_name, msg_class, queue_size) create a handle to publish messages to a topic


    time = rospy.get_time() #current time in float seconds.
    switch = 0
    
    # time controlled loop
    while not rospy.is_shutdown():
      if rospy.get_time() > time +3:
          switch = 1 if switch == 0 else 0
          time = rospy.get_time()
      if switch == 0:    
          pub.publish(False);
      else:
          pub.publish(True);
      rate.sleep()


if __name__ == '__main__':
    main(sys.argv)
