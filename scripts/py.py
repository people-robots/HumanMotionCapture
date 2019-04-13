#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Image

rospy.init_node('aa')
#rospy.Subscriber("/camera/rgb/image_rect_color",Image,imCall)
#rospy.Subscriber("/camera/depth/points",PointCloud2,pcCall)
pub = rospy.Publisher("/camera/rgb/image_rect_color_f",Image,queue_size=1)

def imCall(data):
	#print "IM " + str(data.header.stamp)
	data.header.stamp = rospy.Time.now()
	pub.publish(data)
	
def pcCall(data):
	print "PC " + str(data.header.stamp)
rospy.Subscriber("/camera/rgb/image_rect_color",Image,imCall)
#rospy.Subscriber("/camera/depth/points",PointCloud2,pcCall)

while not rospy.is_shutdown():
	
	rospy.spin()


