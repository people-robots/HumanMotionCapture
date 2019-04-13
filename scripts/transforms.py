#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('HamandiM')
import sys
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf import *
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers


class world_track():
    def __init__(self):
        #rospy.init_node("transforms");
        self.tf_listener = tf.TransformListener();
        self.positions = None;
        self.last_time = rospy.Time.now();
        self.transformation = None;
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.world)
        rospy.Subscriber("/right_hand/position/from_color",PoseStamped, self.position)
        self.position_pub = rospy.Publisher("/right_hand/position/final", PoseStamped, queue_size=10);
        self.trans = np.array([0,0,0]);
        self.rot = np.array([0,0,0,1]);

    def world(self, ar):
        #self.last_time = self.tf_listener.getLatestCommonTime('/camera_depth_frame', '/ar_marker_0')
        if len(ar.markers) > 0:
            #print (ar.markers)
            #trans = ar.markers[0].pose.pose.position
            #trans = [trans.x, trans.y,trans.z]
            #rot   = ar.markers[0].pose.pose.orientation
            #rot = [rot.x, rot.y, rot.z, rot.w]
            #br = tf.TransformBroadcaster()
            #br.sendTransform((trans.x,trans.y,trans.z),(rot.x,rot.y,rot.z,rot.w),self.last_time,"/this_world","/camera_rgb_optical_frame");
            
            #print (trans,rot)
            try:
              self.last_time = self.tf_listener.getLatestCommonTime('/camera_rgb_optical_frame', '/ar_marker_0')
              (trans0, rot0) = self.tf_listener.lookupTransform('/camera_rgb_optical_frame', '/ar_marker_0',self.last_time);
              self.last_time = self.tf_listener.getLatestCommonTime('/camera_rgb_optical_frame', '/ar_marker_1')
              (trans1, rot1) = self.tf_listener.lookupTransform('/camera_rgb_optical_frame', '/ar_marker_1',self.last_time);
              self.last_time = self.tf_listener.getLatestCommonTime('/camera_rgb_optical_frame', '/ar_marker_2')
              (trans2, rot2) = self.tf_listener.lookupTransform('/camera_rgb_optical_frame', '/ar_marker_2',self.last_time);
              self.last_time = self.tf_listener.getLatestCommonTime('/camera_rgb_optical_frame', '/ar_marker_3')
              (trans3, rot3) = self.tf_listener.lookupTransform('/camera_rgb_optical_frame', '/ar_marker_3',self.last_time);
              
              rot = (np.array(rot0)+np.array(rot1)+np.array(rot2)+np.array(rot3))/4;
              trans = (np.array(trans0)+np.array(trans1)+np.array(trans2)+np.array(trans3))/4;
              self.trans = trans;
              self.rot = rot;
              #br = tf.TransformBroadcaster()
              #br.sendTransform(trans,rot,self.last_time,"/this_world","/camera_rgb_optical_frame");
              """
              if self.collected:
                  dist = abs(trans - self.world_p)
                  dist = 2*self.world_sig - dist
                  add = True;
                  for i in range(3):
                      if dist[0,i] >0 :
                          continue
                      else:
                          add = False;
                          break;
                  if add == True:
                      self.world_p_a[self.world_counter,:] = trans;
                      self.world_o_a[self.world_counter,:] = rot;
                      self.world_p = np.average(self.world_p_a,axis=1).T;
                      orientation  = np.average(self.world_o_a,axis=1).T;
                      self.world_counter += 1;
                      br = tf.TransformBroadcaster()
                      br.sendTransform(self.world_p,orientation,self.last_time,
                                       "/this_world_new","/camera_rgb_optical_frame");
              else:
                  self.world_p_a[self.world_counter,:] = trans;
                  self.world_o_a[self.world_counter,:] = rot;
                  self.world_p = np.average(self.world_p_a,axis=1).T;
                  self.world_sig = np.std(self.world_p_a, axis=1).T;
                  self.world_counter += 1;
              if self.world_counter >= 10:
                    self.world_counter = 0;
                    self.collected = True;
            """
            except Exception as e:
              pass
        self.last_time = rospy.Time.now();   
        br = tf.TransformBroadcaster()
        br.sendTransform(self.trans,self.rot,self.last_time,"/this_world","/camera_rgb_optical_frame");
        #for i,trans in enumerate([trans0,trans1,trans2,trans3]):
        #    if (i == 0):
        #        A = np.reshape(np.array(trans),(3,1)).T
        #    else:
        #        A = np.concatenate((A,np.reshape(np.array(trans),(3,1)).T),axis=0)
        
        #y = np.array([[1,1,1,1]]).T

        #out = np.dot(np.dot(np.linalg.inv(np.dot(A.T,A)),A.T),y)
        #out = np.reshape(out,(3,))
        #print (out,A)

            #self.tranformation = np.linalg.inv(self.tf_listener.fromTranslationRotation(trans,rot));
            #print (self.tranformation)

    def position(self, p_msg):
        current_time = p_msg.header.stamp;

        #position = p_msg.pose.position;
        #pose = np.array([[position.x, position.y, position.z, 1]]).T
        #pose = np.dot(self.tranformation,pose)

        #P = PoseStamped();
        P = p_msg;
        
        #print(type(self.last_time)) 
        #P.header.stamp = rospy.Time.from_sec(self.last_time.time());
        P.header.stamp = self.last_time;
        #print (self.last_time,P.header.stamp)
        #P.header.frame_id = p_msg.frame_id;
        #P.pose.position.x = p_msg.pose.position.x

        try:
            P = self.tf_listener.transformPose("/this_world",P)

        except Exception as e:
            pass
        #P.header.stamp = curret_time;
        #P.pose.position.x = pose[0];
        #P.pose.position.y = pose[1];
        #P.pose.position.z = pose[2];
        #P.pose.orientation.z = 1
        #P.pose.orientation.w = 1
        P.header.stamp = current_time;
        br = tf.TransformBroadcaster()
        br.sendTransform([P.pose.position.x,P.pose.position.y,P.pose.position.z]
            ,[0,0,0,1]
                ,current_time,"/hand_tf","/this_world");
        self.position_pub.publish(P);

def main(args):
    rospy.init_node('tranforms', anonymous=True)
    wt = world_track();
    rospy.spin() 

if __name__ == '__main__':
    main(sys.argv)
