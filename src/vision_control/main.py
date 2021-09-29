#!/usr/bin/env python
from std_msgs.msg import String
from math import pi
from sensor_msgs.msg import PointCloud2

import rospy
import pcl
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import time

# ROS node and topics
node_name = 'vision_controller'
obstacle_topic = '/obstacle_detection/obstacle'
background_topic = '/obstacle_detection/background'

# Special signs
tick_sign = u'\u2713'.encode('utf8')
cross_sign = u'\u274c'.encode('utf8')

# Obstacle handler
def obstacle_callback(data):
    # Convert ROS PCL to numpy
    pointcloudXYZ = ros_numpy.numpify(data)
    # Generate an zeros array that match the dimensions of pc
    pointcloudXYZ_np=np.zeros((pointcloudXYZ.shape[0],3))
    # Fill in the data
    pointcloudXYZ_np[:,0]=pointcloudXYZ['x']
    pointcloudXYZ_np[:,1]=pointcloudXYZ['y']
    pointcloudXYZ_np[:,2]=pointcloudXYZ['z']    
    
    print("Size of pointcloud:",pointcloudXYZ_np.shape)


def main():
    # Init node with node_name
    rospy.init_node(node_name)

    # Subscribe obstacle pointcloud
    obstacle_sub = rospy.Subscriber(obstacle_topic, PointCloud2, obstacle_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
