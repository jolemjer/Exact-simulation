#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker


class Node():

    def __init__(self):
        rospy.init_node('robot_info')
        self.pub = rospy.Publisher('position_marker', Marker, queue_size=10)
        rospy.Subscriber('base_pose_ground_truth', Odometry, self.callback)
        self.marker = Marker()
        self.marker.type = Marker.CUBE
        self.marker.header.stamp = rospy.Time(0)
        self.marker.color.r, self.marker.color.g, self.marker.color.b, self.marker.color.a = (0, 1, 0,1)
        self.marker.scale.x, self.marker.scale.y, self.marker.scale.z = 0.1, 0.1, 0.1


    def callback(self, data):
    
        self.marker.header = data.header
        self.marker.pose = data.pose.pose
        self.pub.publish(self.marker)
    

if __name__ == '__main__':
    try:
        node = Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass