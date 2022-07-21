#!/usr/bin/env python3
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image 
from sensor_msgs.msg import CameraInfo
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge 
import struct
import ctypes
import numpy as np
import threading
import copy
from skimage.filters import threshold_otsu
from segment_pointcloud.msg import obj_pointcloud


class pcl_node_handler:
  def __init__(self, roi_topic, depth_image_topic, cam_info_topic):
      self.lock = threading.Lock()
      self.roi = BoundingBoxes()
      self.cvb = CvBridge()
      self.depth_frame = np.ones((4,4))
      self.P_inv = np.eye(4)  # camera matrix inverse
      self.roi_topic = roi_topic
      self.has_roi = False
      self.image_topic = depth_image_topic
      self.cam_info_sub = rospy.Subscriber(cam_info_topic, CameraInfo, self.cb_camera_info)
      self.pub = rospy.Publisher("segmented_pcl", obj_pointcloud, queue_size=1)
      self.class_list = ["person", "chair", "tvmonitor", "bottle", "cell phone"]
      self.depth_acquired = False
      # bgr colors
      self.colors = np.array([[255, 255, 255], [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 0, 255]], dtype=np.uint8)
      # colordict for colored segmented pointcloud in rviz
      self.color_dict = {"person": self.colors[0,:],
                  "chair": self.colors[1,:],
                  "tvmonitor": self.colors[2,:],
                  "bottle": self.colors[3,:],
                  "cell phone": self.colors[4,:]
      }


  def cb_roi(self, msg):
    '''
    callback to update stored region of interest
    nonblocking lock is used to prevent cb_roi() from overwriting while publish_pcl() is reading, 
    but roi_sub keeps running
    params: sensor_msgs/Image
    '''
    if self.lock.acquire(blocking=False): 
      self.roi = msg
      self.has_roi = True
      self.lock.release()

  
  def cb_depth_image(self, msg):
    '''
    callback to update stored depth frame
    nonblocking lock is used to prevent cb_depth_image() from overwriting while publish_pcl() is reading, 
    but image_sub keeps running
    params: darknet_ros_msgs/BoundingBoxes
    '''
    if self.lock.acquire(blocking=False):
      # convert ROS sensor image msg to numpy array
      self.depth_frame = self.cvb.imgmsg_to_cv2(msg) # depth in mm as uint16
      self.depth_acquired = True
      self.lock.release()
  

  def cb_camera_info(self, msg):
    '''
    callback to get camera intrinsics and form inverse of camera matrix
    params: sensor_msgs/camera_info
    '''
    # extract camera intrinsics parameters
    fx = msg.K[0]
    fy = msg.K[4]
    ppx = msg.K[2]
    ppy = msg.K[5]

    # calculate camera matrix inverse
    self.P_inv[0:2,0:3] = np.array([[1/fx, 0, -ppx*fy/(fx*fy)],
                                 [0, 1/fy, -ppy/fy]]) # assume skew is zero

    # unregister camera info subscriber
    self.cam_info_sub.unregister()

    # start bounding box and depth frame subscriber threads
    self.roi_sub = rospy.Subscriber(self.roi_topic, BoundingBoxes, self.cb_roi, queue_size=1)
    self.image_sub = rospy.Subscriber(self.image_topic, Image, self.cb_depth_image, queue_size=1)


  def publish_pcl(self):
    # temporarily lock other threads while copying most recent bounding box and depth frame data 
    self.lock.acquire()
    roi = copy.deepcopy(self.roi)
    depth_frame = copy.deepcopy(self.depth_frame)
    self.lock.release()

    # filter depth pixels between 0.3-10 m and convert to m as float32  
    min_z = 0.3e3 # mm, minimum D435i range
    max_z = 10e3  # mm, maximum D435i range
    # depth_frame = np.where(np.logical_or(depth_frame < min_z, depth_frame > max_z), 0, depth_frame)/1000.0
    depth_frame = np.where(np.logical_or(depth_frame < min_z, depth_frame > max_z), 0, depth_frame)

    # iterate through depth pixels in each bounding box from darknet_ros
    if self.has_roi == True:
      for box in roi.bounding_boxes:
        if box.Class in self.class_list:   # check if object is in class list
          points = [] # list of xyz  points

          # colorization 
          blue = self.color_dict[box.Class][0]
          green = self.color_dict[box.Class][1]
          red = self.color_dict[box.Class][2]
          filler = ctypes.c_uint8(255)  # filler variable for 4 byte rgb, can be any value
          bgr_bytes = struct.pack('BBBB', blue, green, red, filler.value) # pack 4 byte rgb data into bytes object
          bgr = struct.unpack('I', bgr_bytes)[0]  # unpack bytes into uint32 rgb
          if(not self.depth_acquired):  # shouldn't this be before copying the image?
            return
          
          # threshold region of image:
          depth_frame_roi = depth_frame.copy()
          th = threshold_otsu(depth_frame_roi[box.ymin:box.ymax, box.xmin:box.xmax], nbins=65635)
          depth_frame_roi[depth_frame>th] = 0
          depth_frame_roi = depth_frame_roi/1000.0  # convert to m as float 32

          for r in range(box.ymin, box.ymax):
            for c in range(box.xmin, box.xmax):
              # z = depth_frame[r,c]   # get depth info 
              z = depth_frame_roi[r,c]   # get depth info 

              if (z == 0): # skip depth values of zero
                continue

              x = (self.P_inv[0,0]*c + self.P_inv[0,2])*z 
              y = (self.P_inv[1,1]*r + self.P_inv[1,2])*z 
              # add xyz color point to list
              pt = [z, -x, -y, bgr]
              points.append(pt) 
          
          # create pointcloud 
          header = Header()
          header.frame_id = "theia/front_camera_aligned_depth_to_color_frame"
          fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('rgb', 12, PointField.UINT32, 1)]
          pc2 = point_cloud2.create_cloud(header, fields, points)
          obj_pcl = obj_pointcloud()
          obj_pcl.pcl = pc2
          obj_pcl.label = box.Class
          obj_pcl.probability = box.probability
          obj_pcl.y_size = abs(box.ymax - box.ymin)
          obj_pcl.x_size = abs(box.xmax - box.xmin)
          
          self.pub.publish(obj_pcl)
      
      self.roi = BoundingBoxes()
      self.has_roi = False
  
if __name__ == '__main__':
  rospy.init_node("segment_pcl_node")
  roi_topic_name = rospy.get_param("/prm/roi_topic_name")
  depth_image_topic_name = rospy.get_param("/prm/depth_image_topic_name")
  cam_info_topic_name = rospy.get_param("/prm/cam_info_topic_name")
  pub = pcl_node_handler(roi_topic_name, depth_image_topic_name, cam_info_topic_name)
  rate = rospy.Rate(10)
  
  while not rospy.is_shutdown():
    pub.publish_pcl()
    rate.sleep()
