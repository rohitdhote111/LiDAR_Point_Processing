#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <pcl/segmentation/sac_segmentation.h>
#include <chrono>

ros::Publisher pub_filtered;
ros::Publisher pub_ground;
ros::Publisher pub_noise;

// Helper function to compute Euclidean distance from the origin
inline float distanceFromOrigin(const pcl::PointXYZ& point) {
    return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Start measuring time
    auto start = std::chrono::high_resolution_clock::now();

    ROS_INFO("Point cloud received");
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

        // Containers for noise, ground, and filtered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noise(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Define distance thresholds
    float min_distance = 3.0;     // Minimum allowed distance from the origin (in meters)
    float max_distance = 50.0;   // Maximum allowed distance from the origin (in meters)

    // First, filter based on distance from origin to categorize noise points
    for (int i = 0; i < cloud->height; ++i) {    // Traverse by rows (organized point cloud)
        for (int j = 0; j < cloud->width; ++j) { // Traverse by columns
            pcl::PointXYZ point = cloud->at(j, i); // Access the point using 2D indices

            // Compute the Euclidean distance from the origin
            float distance = distanceFromOrigin(point);

            // Apply the distance filter
            if (distance < min_distance || distance > max_distance)
            {
                // Add point to noise points if it's too close or too far
                cloud_noise->points.push_back(point);
            }
            else
            {
                // Add point to cloud_filtered for further processing (non-noise points)
                cloud_filtered->points.push_back(point);
            }
        }
    }

    // RANSAC plane fitting for ground removal
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);  // Distance to consider a point part of the plane (ground)

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    // Extract the inliers (ground plane) and outliers (non-ground points)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);

    // Extract ground points
    extract.setNegative(false);  // Extract inliers (ground)
    extract.filter(*cloud_ground);

    // Extract filtered points (non-ground points)
    extract.setNegative(true);  // Extract outliers (non-ground)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_non_ground);

    // Convert filtered point cloud (non-ground) back to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 output_filtered;
    pcl::toROSMsg(*cloud_non_ground, output_filtered);
    output_filtered.header = input->header;  // Preserve the original header

    // Convert ground point cloud back to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 output_ground;
    pcl::toROSMsg(*cloud_ground, output_ground);
    output_ground.header = input->header;  // Preserve the original header

    // Convert noise point cloud back to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 output_noise;
    pcl::toROSMsg(*cloud_noise, output_noise);
    output_noise.header = input->header;  // Preserve the original header

/*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(80); // Number of neighboring points to analyze for each point
    sor.setStddevMulThresh(3.5); // Standard deviation multiplier for thresholding
    sor.filter(*cloud_filtered);
    sor.setNegative(true);
    sor.filter(*cloud_outliers);
*/



/*    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(30);  // Set radius within which points need neighbors
    ror.setMinNeighborsInRadius(4);  // Minimum neighbors required for a point to be kept

    // Filter the points, removing outliers
    ror.filter(*cloud_filtered);

    // To extract outliers (optional):
    ror.setNegative(true);
    ror.filter(*cloud_outliers);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_filtered;
    pcl::toROSMsg(*cloud_filtered, output_filtered);
    output_filtered.header = input->header;

    sensor_msgs::PointCloud2 output_outliers;
    pcl::toROSMsg(*cloud_outliers, output_outliers);
    output_outliers.header = input->header;

*/

    // Publish filtered point cloud (non-ground)
    pub_filtered.publish(output_filtered);

    // Publish ground points
    pub_ground.publish(output_ground);

    // Publish noise points
    pub_noise.publish(output_noise);


    // Stop measuring time
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    // Print processing time
    ROS_INFO("Point cloud processed in %.4f seconds", elapsed.count());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_filter");
    ros::NodeHandle nh;

    // Subscriber to the input point cloud topic
    ros::Subscriber sub = nh.subscribe("/input_point_cloud", 1, pointCloudCallback);

    // Publishers for filtered (non-ground), ground, and noise point clouds
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/ground_point_cloud", 1);
    pub_noise = nh.advertise<sensor_msgs::PointCloud2>("/noise_point_cloud", 1);

    ros::spin();
    return 0;
}
