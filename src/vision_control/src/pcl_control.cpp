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
#include <nav_msgs/Odometry.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> 
#include "pcl_ros/transforms.h"

// User define
# define M_PI           3.14159265358979323846  

// Node info
std::string ros_node_name = "pcl_controller";                               // Node Name
std::string obstacle_topic = "/obstacle_detection/obstacle";                // PCL Obstacle Topic
std::string navigate_topic = "/navigation_marker";                          // Navigation Vector Topic
std::string control_topic = "/cmd_vel";                                     // Robot Control Topic
std::string odom_topic = "/odom";                                           // Odometry data Topic
ros::Subscriber pcl_sub;                                                    // PCL Obstacle Subsriber              
ros::Subscriber odom_sub;                                                   // Odometry subscriber
ros::Publisher navigate_marker_pub;                                         // Navigation Vector Publisher
ros::Publisher robot_control_pub;                                           // Robot Control Publisher

// Detection parameters
double _y = 0;                                                              // Previous y of a point
int frame_id = 0;                                                           // Frame index
double max_d = 0;                                                           // Maximum gap in the cloud
double gap_mid = 0;                                                         // Mid point of the max gap
double gap_depth = 0;                                                       // Depth of the max gap
const double min_depth = 0.30;                                              // Minimum depth limit
const double right_max = -0.8;                                              // Margin for robot vision of right
const double left_max = 0.8;                                                // Margin for robot vision of left
const double z_cutoff = 1;                                                  // Z cutoff distance in meter
const double gap_min = 0.03;                                                // Minimum gap for robot to pass through it
enum gapType {PCL_ALGO, LEFT_MARGIN, RIGHT_MARGIN,EMPTY,BLOCK, BACK, TURN}; // Gap type
                    

// Control parameters
double linear_x_basic = 0.5;                                                // Basic m/s along X
double angular_z_basic = 0;                                                 // Basuc rad/s along Z
double num_stop = 0;                                                        // Number of stop decision
const double max_num_stop = 20;                                             // Maximum number of stop decision
double rx = 0, ry = 0, rz = 0;                                              // Current euler angle
double _rx = 0, _ry = 0, _rz = 0;                                           // Initial and target euler angle
double qx = 0, qy = 0, qz = 0, qw = 0;                                      // Current quaternion
double backward_delay = 0;                                                  // Backward delay before turning
double forward_delay = 0;                                                   // Forward delay at the very begining (force to move)
const double max_backward_delay = 5;                                        // Allowed backward delay
const double max_forward_delay = 3;                                         // Allowed forward delay
const double max_odom_error = 0.3;                                          // Allowed odom error for rotation
bool euler_lock = false;                                                    // Oodom logging
double total_rz = 0;

// Robot operation schedule
enum taskType {ENTER_ELEVATOR,ROTATE_ITSELF,EXIT_ELEVATOR};                 // Operation sequences for the robot
taskType robot_task = ENTER_ELEVATOR;                                       // Initialize robot operation task

/*
 * Coordinate system in PCL
 * Green : Y (PCL Left/Right)
 * Red   : X (PCL Depth)
 * Blue  : Z (PCL Up/Down)
 * 
*/

// TO-DO: Need to add backoff options in EXIT_ELEVATOR, otherwise it will freeze when no solutions

void quaternion_to_euler()
{
    rx = atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy));
    ry = asin(2*(qw*qy-qz*qx));
    rz = atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Quaternion number
    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;

}

void robot_control(double distance,double direction,gapType gap)
{
    // ROBOT CONTROL
    // Right -> -ve angular z | Left -> +ve angular z
    geometry_msgs::Twist robot_velocity;
    direction = abs(direction);

    // 1. No solution can be found -> STOP
    if(gap == BLOCK)
    {
        robot_velocity.linear.x = 0;
        robot_velocity.angular.z = 0;
        std::cout << "[ROBOT] Action: stop" << std::endl;
        num_stop ++;
        std::cout << "[ROBOT] num_stop: " << num_stop << std::endl;
        // Update scheduled task if the robot cannot find solutions for a long period of time
        if(num_stop>=max_num_stop)
        {
            num_stop = 0;

            robot_task = ROTATE_ITSELF;
        }
    }
    // 2. No obstacles -> FORWARD
    else if(gap == EMPTY)
    {
        robot_velocity.linear.x = linear_x_basic;
        robot_velocity.angular.z = 0;
        std::cout << "[ROBOT] Action: forward" << std::endl;
    }
    //3. A gap on the left -> LEFT
    else if(gap == LEFT_MARGIN)
    {
        robot_velocity.linear.x = linear_x_basic*0.8;
        robot_velocity.angular.z = angular_z_basic*direction+0.8;  
        std::cout << "[ROBOT] Action: Left Extreme" << std::endl;      
    }
    //4. A gap on the right -> RIGHT
    else if(gap == RIGHT_MARGIN)
    {
        robot_velocity.linear.x = linear_x_basic*0.8;
        robot_velocity.angular.z = -1*(angular_z_basic*direction+0.8);
        std::cout << "[ROBOT] Action: Right Extreme" << std::endl;       
    }
    //5. Special for ROTATE_ITSELF -> BACKWARD
    else if(gap == BACK)
    {
        robot_velocity.linear.x = -1*linear_x_basic*0.8;
        robot_velocity.angular.z = 0;
        std::cout << "[ROBOT] Action: backward" << std::endl;       
    }
    //6. Special for ROTATE_ITSELF -> ROTATION
    else if(gap == TURN)
    {
        robot_velocity.linear.x = 0;
        robot_velocity.angular.z = 1.5;
        std::cout << "[ROBOT] Action: rotate" << std::endl;       
    }
    //7. Follow PCL 
    else
    {
        if(gap_mid>0)
        {
            // Left
            robot_velocity.linear.x = linear_x_basic;
            robot_velocity.angular.z = angular_z_basic*(1.1+direction);
            std::cout << "[ROBOT] Action: Left-PCL" << std::endl;
        }else{
            // Right
            robot_velocity.linear.x = linear_x_basic;
            robot_velocity.angular.z = -1*angular_z_basic*(1.1+direction);
            std::cout << "[ROBOT] Action: Right-PCL" << std::endl;
        }
    }

    // Publish message 
    robot_control_pub.publish(robot_velocity);
}

void cloud_callback (const sensor_msgs::PointCloud2 &cloud_msg)
{
    std::map<double,double> linear_scan;
    std::map<double,double>::iterator iter;

    // Left Margin Marker
    visualization_msgs::Marker left_limit_marker;
    left_limit_marker.header.frame_id = "/camera_link";
    left_limit_marker.header.stamp = ros::Time::now();
    left_limit_marker.ns = "basic_shapes";
    left_limit_marker.id = 0;
    left_limit_marker.type = visualization_msgs::Marker::ARROW;
    left_limit_marker.action = visualization_msgs::Marker::ADD;
    left_limit_marker.lifetime = ros::Duration(); 

    // Left Margin Marker end point
    geometry_msgs::Point left_limit_pt;
    left_limit_pt.x = 1.0;
    left_limit_pt.y = left_max;
    left_limit_pt.z = 0;

    // Right Margin Marker
    visualization_msgs::Marker right_limit_marker;
    right_limit_marker.header.frame_id = "/camera_link";
    right_limit_marker.header.stamp = ros::Time::now();
    right_limit_marker.ns = "basic_shapes";
    right_limit_marker.id = 1;
    right_limit_marker.type = visualization_msgs::Marker::ARROW;
    right_limit_marker.action = visualization_msgs::Marker::ADD;
    right_limit_marker.lifetime = ros::Duration();       

    // Right Margin Marker end point
    geometry_msgs::Point right_limit_pt;
    right_limit_pt.x = 1.0;
    right_limit_pt.y = right_max;
    right_limit_pt.z = 0;

    // Navigation Marker
    visualization_msgs::Marker navigation_marker;
    navigation_marker.header.frame_id = "/camera_link";
    navigation_marker.header.stamp = ros::Time::now();
    navigation_marker.ns = "basic_shapes";
    navigation_marker.id = 2;
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

    // Robot Task Statement
    std::cout << "[SYSTEM] Frame: " << frame_id << std::endl;
    if(robot_task == ENTER_ELEVATOR)
    {
        std::cout << "[Schedule] Current Task: ENTER ELEVATOR" << std::endl;
    }else if(robot_task == ROTATE_ITSELF)
    {
        std::cout << "[Schedule] Current Task: ROTATE ITSELF" << std::endl;
    }else{
        std::cout << "[Schedule] Current Task: EXIT ELEVATOR" << std::endl;
    }

    // Only being used when the robot enters the elevator / leave the elevator
    if((robot_task == ENTER_ELEVATOR)||(robot_task == EXIT_ELEVATOR))
    {
        // Store all points data(y,x) into a vector for path planning, eliminate y axis
        bool valid_linear_scan = false;
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

            if(pointcloudX < z_cutoff)
            {
                // Only consider Y axis [left & right] and X axis [depth]
                linear_scan.insert(std::make_pair(pointcloudY,pointcloudZ));
                valid_linear_scan = true;
            }
        }

        // If no valid linear_scan, just go forward
        if((valid_linear_scan)&&(forward_delay == max_forward_delay))
        {
            max_d = 0;
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
                    gap_mid = (iter->first+_y)*0.5; 
                    gap_depth = (iter->second);     
                }

                
                // Update previous value
                _y = iter->first;

            }

            // Calculating marginal spaces
            double left_margin_d = 0.0;
            double right_margin_d = 0.0;
            double start_y = 0.0;
            double start_z = 0.0;
            double end_y = 0.0;
            double end_z = 0.0;

            iter = linear_scan.begin();
            left_margin_d = abs(iter->first - left_max);
            start_y = iter->first;
            start_z = iter->second;

            iter = linear_scan.end();
            iter --;
            right_margin_d = abs(right_max - iter->first);
            end_y = iter->first;
            end_z = iter->second;

            max_d = max_d*10;

            // Display all calculated results
            std::cout << "[CALCULATION] Largest Gap: " << max_d << " | Right Margin: " << right_margin_d << " | Left Margin: " << left_margin_d << std::endl;

            // Determine max_d or marginal spaces should be used
            gap_depth = abs(gap_depth);
            if((right_margin_d>left_margin_d)&&(right_margin_d>max_d))
            {
                gap_mid = (right_max+end_y)*0.5;
                //gap_depth = abs(end_z);
                // Check the gap is enought to pass through
                std::cout << "[VISION] Right margin space -> mid: " << gap_mid << " | depth: " << gap_depth << std::endl;
                if((right_margin_d>gap_min)&&(gap_depth>=min_depth))
                {
                    // Show determined solution
                    robot_control(right_margin_d,gap_mid,RIGHT_MARGIN);
                }else{
                    std::cout << "[VISION] NO SOLUTIONS!" << std::endl;
                    robot_control(right_margin_d,gap_mid,BLOCK);
                }
            }
            else if((left_margin_d>right_margin_d)&&(left_margin_d>max_d))
            {
                gap_mid = (left_max+start_y)*0.5;
                // gap_depth = abs(start_z);
                // Check the gap is enought to pass through
                std::cout << "[VISION] Left margin space -> mid: " << gap_mid << " | depth: " << gap_depth << std::endl;
                if((left_margin_d>gap_min)&&(gap_depth>=min_depth))        
                {
                    // Show determined solution
                    robot_control(left_margin_d,gap_mid,LEFT_MARGIN);            // Show determined solution
                }else{
                    std::cout << "[VISION] NO SOLUTIONS!" << std::endl;
                    robot_control(left_margin_d,gap_mid,BLOCK);
                }
            }else
            {
                // When depth is below min_depth, the robot may have change to collide with some obstacles
                std::cout << "[VISION] PCL space -> mid: " << gap_mid << " | depth: " << gap_depth << std::endl;
                if(gap_depth>=min_depth)
                {
                    // Show determined solution
                    robot_control(max_d,gap_mid,PCL_ALGO);
                }else{
                    std::cout << "[VISION] NO SOLUTIONS!" << std::endl;
                    robot_control(max_d,gap_mid,BLOCK);
                }
            }
            std::cout << std::endl;
            // Publish navigation marker
            arrow_end.x = 1.0;
            arrow_end.y = gap_mid;
            arrow_end.z = 0;  
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

        }else{
            std::cout << "[VISION] Out of range: FORWARD!" << std::endl;
            std::cout << std::endl;
            robot_control(1000,0,EMPTY);
            
            // Publish navigation marker
            arrow_end.x = 1.0;
            arrow_end.y = 0;
            arrow_end.z = 0;  
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
            forward_delay ++;
        }

    }

    // Robot self rotation
    if(robot_task == ROTATE_ITSELF)
    {
        //TO-DO: Robot self rotation
        backward_delay++;
        if(backward_delay<max_backward_delay)
        {
            robot_control(0,0,BACK);
        }else{
            // Rotate robot based on odometry
            quaternion_to_euler();
            if (!euler_lock)
            {
                _rx = rx;
                _ry = ry;
                _rz = rz + M_PI;
                euler_lock = !euler_lock;
            }     
            total_rz += abs(rz-_rz);
            if(total_rz<=35)
            {
                std::cout << "[ODOM] Initial euler: rx= " << _rx << "| ry= " << _ry<< "| rz= " << _rz-M_PI << std::endl;
                std::cout << "[ODOM] Target euler: rx= " << _rx << "| ry= " << _ry<< "| rz= " << _rz << std::endl;
                std::cout << "[ODOM] Current euler: rx= " << rx << "| ry= " << ry<< "| rz= " << rz << std::endl;
                std::cout << "[ODOM] total drz: " << total_rz << std::endl;   
                std::cout << std::endl;   
                robot_control(0,0,TURN);
            }else{
                robot_task = EXIT_ELEVATOR;
            }
        }
    }

    // Publish right margin marker 
    right_limit_marker.points.push_back(arrow_start);
    right_limit_marker.points.push_back(right_limit_pt);
    right_limit_marker.scale.x = 0.03;
    right_limit_marker.scale.y = 0.05;
    right_limit_marker.scale.z = 0.1;
    right_limit_marker.color.r = 1.0;
    right_limit_marker.color.g = 0.4;
    right_limit_marker.color.b = 1.0;
    right_limit_marker.color.a = 1.0;
    navigate_marker_pub.publish(right_limit_marker);   

    // Publish left margin marker 
    left_limit_marker.points.push_back(arrow_start);
    left_limit_marker.points.push_back(left_limit_pt);
    left_limit_marker.scale.x = 0.03;
    left_limit_marker.scale.y = 0.05;
    left_limit_marker.scale.z = 0.1;
    left_limit_marker.color.r = 1.0;
    left_limit_marker.color.g = 0.4;
    left_limit_marker.color.b = 1.0;
    left_limit_marker.color.a = 1.0;
    navigate_marker_pub.publish(left_limit_marker);  

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

    // Create a ROS subscriber to get odometry data
    odom_sub = nh.subscribe(odom_topic, 1000, odomCallback);

    // Create a ROS publisher to publish control intention
    navigate_marker_pub = nh.advertise<visualization_msgs::Marker>(navigate_topic,1);

    // Create a ROS publisher to publish cmd_vel
    robot_control_pub = nh.advertise<geometry_msgs::Twist>(control_topic,1000);

    // Spin
    ros::spin ();
}