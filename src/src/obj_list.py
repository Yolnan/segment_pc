import rospy
from segment_pointcloud.msg import obj_centroid_data, obj_centroid_list

class obj_list():
    def __init__(self):
        self.obj_seen = {}
        self.pub = rospy.Publisher('/obj_list',obj_centroid_list,queue_size=1)
        self.sub = rospy.Subscriber('/pcl_centers',obj_centroid_data,self.callback)
        self.pubdata = obj_centroid_list()


    def callback(self, data):
        if data not in self.obj_seen:
            self.obj_seen.add(data)
        
    def publisher(self):
        self.pub.publish(tuple(self.obj_seen))

if __name__ == "__main__":
    rospy.init_node("Object_List_Maintainer")
    lister = obj_list()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        lister.publisher()
        rate.sleep()
