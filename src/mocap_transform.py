#!/usr/bin/env python3

from ctypes import pointer
import rospy # Python library for ROS
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import ros_numpy
import numpy as np
import threading
import copy


class Mocap_transform():
  def __init__(self,theia_topic,obj_topic):
    self.theia_pos = PoseStamped()
    self.obj_pos = PoseStamped()
    self.lock = threading.Lock()
    self.theia_sub = rospy.Subscriber(theia_topic,PoseStamped,self.cb_theia,queue_size=1)
    self.obj_sub = rospy.Subscriber(obj_topic,PoseStamped,self.cb_obj,queue_size=1)
    self.pub = rospy.Publisher("Mocap_transform",PointStamped,queue_size=1)
  
  def cb_theia(self,msg):
    '''
    Callback to update the global coordinate of theia
    Nonblocking lock to prevent overwriting publisher
    '''
    if self.lock.acquire(blocking=False): 
      self.theia_pos = msg
      self.lock.release()
  
  def cb_obj(self,msg):
    '''
    Callback to update global coordinate of the object
    Nonblocking lock to prevent overwriting publisher
    '''
    if self.lock.acquire(blocking=False): 
      self.obj_pos = msg
      self.lock.release()
  
  def publish_transform(self):
    self.lock.acquire()
    theia_pos = copy.deepcopy(self.theia_pos)
    obj_pos = copy.deepcopy(self.obj_pos)
    self.lock.release()
    transform_pt = PointStamped()
    transform_pt.header.stamp = rospy.Time.now()
    transform_pt.header.frame_id = "/transformed_coord"
    transform_pt.point.x = obj_pos.pose.position.x - theia_pos.pose.position.x
    transform_pt.point.y = obj_pos.pose.position.y - theia_pos.pose.position.y
    transform_pt.point.z = obj_pos.pose.position.z - theia_pos.pose.position.z
    self.pub.publish(transform_pt)



if __name__ == '__main__':
  # initialize node with name
  rospy.init_node('mocap_transform', anonymous=True)
  theia_topic = "/theia/vrpn_client_node/Chair/pose"
  obj_topic = "/theia/vrpn_client_node/Theia/pose"
  pub = Mocap_transform(theia_topic,obj_topic)
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    pub.publish_transform()
    rate.sleep()