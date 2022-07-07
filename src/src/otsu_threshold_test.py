#!/usr/bin/env python3
import numpy as np
from skimage.filters import threshold_otsu
import rospy
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 


def callback(msg):

    cvb = CvBridge()
    pub = rospy.Publisher('/thresholded_depth',Image,queue_size=1)
    depth_img = cvb.imgmsg_to_cv2(msg).copy()
    th = threshold_otsu(depth_img, nbins=65635)
    print(type(th))
    depth_img[depth_img>th] = 0
    out_msg = cvb.cv2_to_imgmsg(depth_img, encoding="passthrough")
    pub.publish(out_msg)


def receive_message():
    # initialize node with name
    rospy.init_node('otsu_threshold_node', anonymous=True)

    # Specify topics to subscribe to
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback=callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    receive_message()