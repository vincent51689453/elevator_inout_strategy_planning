// C++ STL
#include <iostream>
#include <string>
#include <vector>

// User libraries
#include "point_processing.h"

// ROS 
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> 
#include "pcl_ros/transforms.h"

// Node info
std::string ros_node_name = "pcl_controller";
std::string obstacle_topic = "/obstacle_detection/obstacle";

// User parameters
int frame_id = 0;


void cloud_callback (const sensor_msgs::PointCloud2 &cloud_msg)
{
    std::cout << "Index: " << frame_id << std::endl;

    // Convert pointcloud2 to pointcloud 
    // Pointcloud is the specific datatype for actual usage
    // Pointcloud2 is a general representation containing a header for ROS
    sensor_msgs::PointCloud obstacle_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg,obstacle_cloud);

    // Test to print point 0 (x,y,z).
    std:: cout << "Point 0 x:" << obstacle_cloud.points[0].x << " y:" << obstacle_cloud.points[0].y << " z:" << obstacle_cloud.points[0].z << std::endl; 




    frame_id++;
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