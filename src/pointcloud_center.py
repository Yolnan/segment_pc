#!/usr/bin/env python3
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped as ps
import ros_numpy
import numpy as np


 
def callback(data):
  pub = rospy.Publisher('/centroid',ps,queue_size=1)
  xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data,remove_nans=True)
  xyz = xyz[xyz[:,2]<np.quantile(xyz[:,2],0.99,0),:]
  xyz = xyz[xyz[:,2]>np.quantile(xyz[:,2],0.01,0),:]
  xyz_mean = np.mean(xyz,axis=0)

  point = ps()
  if not rospy.is_shutdown():
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "/odom"
    point.point.x = xyz_mean[0]
    point.point.y = xyz_mean[1]
    point.point.z = xyz_mean[2]
    pub.publish(point)


  



  
      
def receive_message():
  # initialize node with name
  rospy.init_node('pointcloud_center', anonymous=True)
  
  # Specify topics to subscribe to
  rospy.Subscriber("segmented_pcl", PointCloud2, callback=callback)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  
if __name__ == '__main__':
  receive_message()