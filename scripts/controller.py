#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('HamandiM')
import sys
import rospy
import cv2
import random
import numpy as np
import os
import os.path
#import os.makedirs
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import AccelStamped
from std_msgs.msg import Bool
from std_msgs.msg import String
import matplotlib.pyplot as plt
import rospkg
from metadata import *

class controller_node:

  def __init__(self):    
      self.button_last = False;
      self.file_name ='';
      self.target_dir = "";
      self.recorder = [];
      self.font = cv2.FONT_HERSHEY_SIMPLEX;
      self.rows = ['A','B','C','D','E','F','G','H','I','J','K'];
      self.cols = ['1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','16'];
      self.empty_image = np.ones((200,400,3),np.uint8)*255;
      cv2.namedWindow("The Target");
      self.is_displayed = False;
      self.pack = rospkg.RosPack();
      self.path = self.pack.get_path("human_motion_capture");
      self.counter = 0;
      #self.fig = plt.figure(figsize=(3,2));
      #self.ax = self.fig.add_subplot(111);
      #self.text = self.ax.text(0.25,0.25,str('0'),fontsize=60)
      #self.fig.show(0)
      #self.b;
      self.target_pub =rospy.Publisher("/target_loc",String,
              queue_size=10);
      self.change_target();
      self.button_sub = rospy.Subscriber("/button",Bool, self.button_callback);
      self.loc_sub = rospy.Subscriber("/right_hand/position/final",PoseStamped, self.vision_callback);
      self.acc_sub = rospy.Subscriber("/right_hand/acceleration",AccelStamped, self.acc_callback);

  def button_callback(self, button):
      #print (button)
      if not button.data:
          if self.button_last == True:
              self.button_last = False;
              self.record();
              self.change_target();
              self.counter += 1;
              self.recorder = [];
          else:
              self.button_last = False;
      else:
          if self.button_last == False:
              self.target_pub.publish(self.target_dir);
              self.button_last = True;

  def vision_callback(self, position):
      if self.button_last == True:
          new_data = {};
          new_data['type'] = 'position';
          new_data['stamp'] = position.header.stamp.secs + float(position.header.stamp.nsecs)/(10 ** (9));
          new_data['x'] = position.pose.position.x;
          new_data['y'] = position.pose.position.y;
          new_data['z'] = position.pose.position.z;
          self.recorder.append(new_data);

  def acc_callback(self, acc):
      if self.button_last == True:
          new_data = {};
          new_data['type'] = 'acc';
          stamp = rospy.Time.now()
          new_data['stamp'] = stamp.secs + float(stamp.nsecs)/(10 ** 9);
    #new_data['stamp'] = acc.header.stamp.secs + float(acc.header.stamp.nsecs)/(10 ** 9);
          new_data['ax'] = acc.accel.linear.x;
          new_data['ay'] = acc.accel.linear.y;
          new_data['az'] = acc.accel.linear.z;
          new_data['gx'] = acc.accel.angular.x;
          new_data['gy'] = acc.accel.angular.y;
          new_data['gz'] = acc.accel.angular.z;
          self.recorder.append(new_data);

  def record(self):
      data = {};
      data['data'] = self.recorder;
      data_old = [];
      if os.path.isfile(self.file_name):
          with open(self.file_name, 'r') as f:
              print ("check the savings")
              data_old = json.load(f);
              seq = len(data_old) + 1;
              data['seq'] = seq;
              data_old.append(data)
          with open(self.file_name, 'w') as f:
          #print (data_old)
              json.dump( data_old, f, indent=4, sort_keys=True);

      else:
          data['seq'] = 1
          data_old = [data]
          with open(self.file_name,'w') as f:
        #print (data_old)
            json.dump( data_old, f, indent=4, sort_keys=True);

  def change_target(self):
      metadata_file = self.path+"/data/metadata.json"
      target = select_random(4,metadata_file)
      if target is None:
        target = "END"
      #target_row = random.choice(self.rows);
      #target_col = random.choice(self.cols);
      b = np.copy(self.empty_image);
      cv2.putText(b,str(target),(80,150),self.font, 5, (0,0,0));
      self.b = b;
      print (str(target) +" counter: " +str(self.counter))
      #plt.close()
      #self.fig.destroy();
      #fig = plt.figure(figsize=(3,2));
      #ax = fig.add_subplot(111);
      #self.text.remove()
      #text = ax.text(0.25,0.25,str(target_row+target_col),fontsize=60)
      #fig.show(0)
      count = 0;
      target_dir = str(self.path + "/data/images_points/"+ target
              +"_"+str(count));
      while (True):
          if os.path.isdir(target_dir):
              count += 1;
              target_dir = str(self.path + "/data/images_points/"+ target +"_"+ str(count));
          else:
              os.makedirs(target_dir);
              break;
      #self.target_pub.publish(target_dir);
      self.target_dir = target_dir;
      self.file_name = str(self.path + "/data/" + target+'.json')
      self.is_displayed = False;
      meta_counter(target,metadata_file);
      #self.display_target()

  def display_target(self):
      self.is_displayed = True
      cv2.imshow("The Target", self.b);
      cv2.waitKey(5);


def main(args):
    rospy.init_node('Control_node', anonymous=True)
    recorder_node = controller_node()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if recorder_node.is_displayed == False:
            recorder_node.display_target();
        rospy.sleep(0.5)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
