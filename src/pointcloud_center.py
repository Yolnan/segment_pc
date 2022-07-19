#!/usr/bin/env python3
# Import the necessary libraries
import rospy # Python library for ROS
from geometry_msgs.msg import PointStamped as ps
from segment_pointcloud.msg import obj_centroid_data, obj_pointcloud
import ros_numpy
import numpy as np

pub = rospy.Publisher('/pcl_centers',obj_centroid_data,queue_size=1)

def callback(data):
  try:
    xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data.pcl,remove_nans=True)
    xyz = xyz[xyz[:,2]<np.quantile(xyz[:,2],0.99,0),:]
    xyz = xyz[xyz[:,2]>np.quantile(xyz[:,2],0.01,0),:]
    xyz_mean = np.mean(xyz,axis=0)

    out = obj_centroid_data()
    if not rospy.is_shutdown():
      out.header.stamp = rospy.Time.now()
      out.header.frame_id = "theia/front_camera_aligned_depth_to_color_frame"
      out.centroid.x = xyz_mean[0]
      out.centroid.y = xyz_mean[1]
      out.centroid.z = xyz_mean[2]
      out.label = data.label
      out.probability = data.probability
      out.x_size = data.x_size
      out.y_size = data.y_size
      pub.publish(out)
  except IndexError:
    print(data)


  



  
      
def receive_message():
  # initialize node with name
  rospy.init_node('pointcloud_center', anonymous=True)
  
  # Specify topics to subscribe to
  rospy.Subscriber("segmented_pcl", obj_pointcloud, callback=callback)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  
if __name__ == '__main__':
  receive_message()