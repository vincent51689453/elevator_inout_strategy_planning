// C++ STL
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <math.h>
#include <time.h>

// ROS 
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> 
#include "pcl_ros/transforms.h"

// Node info
std::string ros_node_name = "pcl_controller";                   // Node Name
std::string obstacle_topic = "/obstacle_detection/obstacle";    // PCL Obstacle Topic
std::string navigate_topic = "/navigation_marker";              // Navigation Vector Topic
ros::Publisher navigate_marker_pub;                             // PCL Obstacle Subsriber
ros::Subscriber pcl_sub;                                        // Navigation Vector Publisher

// User parameters
double _y = 0;                                                  // Previous y of a point
int frame_id = 0;                                               // Frame index
double max_d = 0;                                               // Maximum gap
double mid_y = 0;                                               // mid_y of the gap
double mid_x = 0;                                               // mid_x of the gap (depth)
const double robot_Width = 0.55;                                // Robot Width

/*
 * Coordinate system in PCL
 * Green : Y (PCL Left/Right)
 * Red   : X (PCL Depth)
 * Blue  : Z (PCL Up/Down)
 * 
*/




void cloud_callback (const sensor_msgs::PointCloud2 &cloud_msg)
{
    std::map<double,double> linear_scan;
    std::map<double,double>::iterator iter;

    // Navigation Marker
    visualization_msgs::Marker navigation_marker;
    navigation_marker.header.frame_id = "/camera_link";
    navigation_marker.header.stamp = ros::Time::now();
    navigation_marker.ns = "basic_shapes";
    navigation_marker.id = 0;
    navigation_marker.type = visualization_msgs::Marker::ARROW;
    navigation_marker.action = visualization_msgs::Marker::ADD;
    navigation_marker.lifetime = ros::Duration();
    
    //navigation_marker starting points
    geometry_msgs::Point arrow_start;
    arrow_start.x = arrow_start.y = arrow_start.z = 0;

    //navigation_marker starting points
    geometry_msgs::Point arrow_end;
    arrow_end.x = arrow_end.y = arrow_end.z = 0;

    
    // Convert pointcloud2 to pointcloud 
    // Pointcloud is the specific datatype for actual usage
    // Pointcloud2 is a general representation containing a header for ROS
    sensor_msgs::PointCloud obstacle_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_msg,obstacle_cloud);

    // Store all points data(y,x) into a vector for path planning, eliminate y axis
    for(int i=0;i<obstacle_cloud.points.size();i++)
    {
        //Show XYZ data
        //std:: cout << "[DEBUG Point #" << i << std::endl;
        //std:: cout << "x:" << obstacle_cloud.points[i].x << " y:" << obstacle_cloud.points[i].y << " z:" << obstacle_cloud.points[i].z << std::endl; 
        double y = obstacle_cloud.points[i].y;
        double x = obstacle_cloud.points[i].x;
        double z = obstacle_cloud.points[i].z;

        // Convert pointcloud2 XYZ to pointcloud XYZ
        double pointcloudX = z;
        double pointcloudY = -1*x;
        double pointcloudZ = -1*y;

        // Only consider Y axis [left & right] and X axis [depth]
        linear_scan.insert(std::make_pair(pointcloudY,pointcloudZ));
    }

    // Searching for 'GAP' between obstacles
    for(iter=linear_scan.begin();iter!=linear_scan.end();iter++)
    {
        double d = 0.0;
        // Find distance between previous point and current point along Y
        (iter->first>_y)?d = iter->first-_y:d=0;


        // Find maximum d
        if(d>max_d)
        {
            max_d = d;
            mid_y = (iter->first+_y)*0.5;
            // Update navigation_marker end points
            arrow_end.x = 0.6;
            arrow_end.y = mid_y;
            arrow_end.z = 0;        
        }

        // Update previous value
        _y = iter->first;

    }

            
    //Publish navigation marker
    navigation_marker.points.push_back(arrow_start);
    navigation_marker.points.push_back(arrow_end);
    navigation_marker.scale.x = 0.03;
    navigation_marker.scale.y = 0.05;
    navigation_marker.scale.z = 0.1;
    navigation_marker.color.r = 0.2;
    navigation_marker.color.g = 1.0;
    navigation_marker.color.b = 1.0;
    navigation_marker.color.a = 1.0;
    navigate_marker_pub.publish(navigation_marker);
     
    frame_id++;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, ros_node_name);
    ros::NodeHandle nh;
    std::cout << "[SYSTEM] " << ros_node_name << " is started!" << std::endl;

    // Create a ROS subscriber to get obstacles
    pcl_sub = nh.subscribe (obstacle_topic, 1, cloud_callback);

    // Create a ROS publisher to publish control intention
    navigate_marker_pub = nh.advertise<visualization_msgs::Marker>(navigate_topic,1);

    // Spin
    ros::spin ();
}