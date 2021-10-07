// C++ STL
#include <iostream>
#include <string>

// ROS 
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> 

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> 
#include "pcl_ros/transforms.h"

// Node info
std::string ros_node_name = "pcl_controller";
std::string obstacle_topic = "/obstacle_detection/obstacle";

void cloud_callback (sensor_msgs::PointCloud2 cloud_msg)
{

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ros_node_name");
    ros::NodeHandle nh;

    // Create a ROS subscriber to get obstacles
    ros::Subscriber sub = nh.subscribe (obstacle_topic, 1, cloud_callback);

    // Spin
    ros::spin ();
}