#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('HamandiM')
import sys
import rospy
from std_msgs.msg import Bool

def main(args):
    rospy.init_node('button', anonymous=True)
    rate = rospy.Rate(1)
    pub = rospy.Publisher('/button',Bool, queue_size=10);
    time = rospy.get_time()
    switch = 0
    
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
