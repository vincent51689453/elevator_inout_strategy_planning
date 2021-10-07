// C++ STL
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <math.h>

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
std::map<double,double> linear_scan;
std::map<double,double>::iterator iter;
std::vector<double> distances;
double _x = 0;
double _z = 0;


void cloud_callback (const sensor_msgs::PointCloud2 &cloud_msg)
{
    std::cout << "Index: " << frame_id << std::endl;

    // Convert pointcloud2 to pointcloud 
    // Pointcloud is the specific datatype for actual usage
    // Pointcloud2 is a general representation containing a header for ROS
    sensor_msgs::PointCloud obstacle_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg,obstacle_cloud);

    // Store all points data(x,z) into a vector for path planning, eliminate y axis
    for(int i=0;i<obstacle_cloud.points.size();i++)
    {
        //Show XYZ data
        //std:: cout << "[DEBUG Point #" << i << std::endl;
        //std:: cout << "x:" << obstacle_cloud.points[i].x << " y:" << obstacle_cloud.points[i].y << " z:" << obstacle_cloud.points[i].z << std::endl; 
        double x = obstacle_cloud.points[i].x;
        double z = obstacle_cloud.points[i].z;
        linear_scan.insert(std::make_pair(x,z));

       
        //Find distance betwwen previous point and current point
        double d = 0.0;
        d = sqrt((x-_x)*(x-_x)+(z-_z)*(z-_z));    //WRONG
        std::cout << d << std::endl;
        distances.push_back(d);

        //Search for maximum distance in vector<double>dsitances;
        std::cout << "Maximum gap along X: " << distances[distances.size()-1] << std::endl;

        //Update previous values by current values
        _x = x;
        _z = z;
    }

    frame_id++;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, ros_node_name);
    ros::NodeHandle nh;
    std::cout << "[SYSTEM] " << ros_node_name << " is started!" << std::endl;

    // Create a ROS subscriber to get obstacles
    ros::Subscriber sub = nh.subscribe (obstacle_topic, 1, cloud_callback);

    // Spin
    ros::spin ();
}