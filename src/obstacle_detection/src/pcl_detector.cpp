#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> 

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> 
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

// Voxel Grid & Pass through filters
float voxel_leaf_szie = 0.01;   //in terms of meter
int z_pass_min = -1;            //in terms of meter
int z_pass_max = 2.5;           //in terms of meter

// Segmentation
float plane_dist_thresh = 0.0001;
int plane_max_iter = 50;

// Eculidean Distance Extraction
float cluster_tol = 0.08;
int cluster_min_size = 100;
int cluster_max_size = 50000;

ros::Publisher obstacle_pub;
ros::Publisher background_pub;

void cloud_callback (sensor_msgs::PointCloud2 cloud_msg)
{
    //Create a new point cloud for storing raw input
    pcl::PointCloud<pcl::PointXYZ> raw_input_cloud;
    //Convert ros msg into PCL data
    pcl::fromROSMsg(cloud_msg, raw_input_cloud);

    //1. VOXEL GRID FILTER [START]
    //Pointer for raw input and output of voxel filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> (raw_input_cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud (cloud_ptr);
    voxel_filter.setLeafSize (voxel_leaf_szie,voxel_leaf_szie,voxel_leaf_szie);
    voxel_filter.filter (*cloud_voxel_filtered);
    //1. VOXEL GRID FILTER [END]

    //2. PASS THROUGH FILTER [START]
    //Create a new point cloud for storing filter output
    pcl::PointCloud<pcl::PointXYZ> zf_cloud;

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_voxel_filtered);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_pass_min,z_pass_max);
    pass_z.filter(zf_cloud);
    //2. PASS THROUGH FILTER [END]

    //3. PLANE SEGMENTATION [START]
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (plane_max_iter);
    seg.setDistanceThreshold (plane_dist_thresh);

    // Segment the largest planar component from the cropped cloud
    seg.setInputCloud (cropped_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
        //break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cropped_cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    //3. PLANE SEGMENTATION [END]

    // Publish the data
    sensor_msgs::PointCloud2::Ptr output_msg(new sensor_msgs::PointCloud2);

    // Convert PCL to ros msg
    pcl::toROSMsg(*cloud_f,*output_msg);
    obstacle_pub.publish (output_msg);

    pcl::toROSMsg(*cloud_plane,*output_msg);
    background_pub.publish(output_msg);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "obstacle_detector");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    obstacle_pub = nh.advertise<sensor_msgs::PointCloud2> ("/obstacle_detection/obstacle", 1);
    background_pub = nh.advertise<sensor_msgs::PointCloud2> ("/obstacle_detection/background", 1); 
       
    // Spin
    ros::spin ();
}