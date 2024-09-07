#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <chrono>  // For timing

ros::Publisher pub_filtered;
ros::Publisher pub_ground;
ros::Publisher pub_noise;

// Helper function to compute Euclidean distance from the origin
inline float distanceFromOrigin(const pcl::PointXYZ& point) {
    return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

// Global variables for RANSAC plane parameters
int pointCloudCounter = 0;         // Counter for the number of point clouds processed
int ransacFrequency = 5;           // Frequency for recomputing the RANSAC plane

pcl::ModelCoefficients::Ptr cached_coefficients(new pcl::ModelCoefficients);  // Cached plane coefficients
bool ransac_initialized = false;   // Flag to check if RANSAC plane has been computed once

void computeRansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered) {
    // RANSAC segmentation for plane fitting
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);  // Distance threshold for RANSAC

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  // Inliers for the plane

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *cached_coefficients);  // Perform segmentation and store coefficients

    if (inliers->indices.empty()) {
        ROS_WARN("No plane found during RANSAC segmentation.");
    }

    ROS_INFO("RANSAC plane computed and cached.");
    ransac_initialized = true;  // Mark RANSAC as initialized
}

// Function to classify ground and non-ground points using cached plane parameters
void classifyUsingCachedPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ground,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_non_ground) {
    // If RANSAC hasn't been computed, return
    if (!ransac_initialized) {
        ROS_WARN("RANSAC plane not initialized.");
        return;
    }

    // Extract points using cached plane coefficients
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Iterate through each point and determine if it belongs to the plane
    for (int i = 0; i < cloud_filtered->height; ++i) {  // Iterate through rows
        for (int j = 0; j < cloud_filtered->width; ++j) {  // Iterate through columns
            pcl::PointXYZ point = cloud_filtered->at(j, i);  // Access point using 2D indexing

        // Plane equation: ax + by + cz + d = 0
        float distance = std::fabs(cached_coefficients->values[0] * point.x +
                                   cached_coefficients->values[1] * point.y +
                                   cached_coefficients->values[2] * point.z +
                                   cached_coefficients->values[3]);

        // Points within the threshold are considered as ground points
        if (distance <= 0.1) {
            inliers->indices.push_back(i);
              }
         }
       }

    // Extract ground and non-ground points based on inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);

    extract.setNegative(false);  // Extract inliers (ground points)
    extract.filter(*cloud_ground);

    extract.setNegative(true);   // Extract outliers (non-ground points)
    extract.filter(*cloud_non_ground);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    auto start_time = std::chrono::high_resolution_clock::now();  // Start timing

    ROS_INFO("Point cloud received");

    pointCloudCounter++;  // Increment point cloud counter

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // Containers for filtered, ground, and noise points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noise(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>);

    // Distance-based noise filtering
    float min_distance = 3.0;  // Minimum distance (3 meters)
    float max_distance = 50.0; // Maximum distance (50 meters)

    for (int i = 0; i < cloud->height; ++i) {  // Iterate over rows
        for (int j = 0; j < cloud->width; ++j) {  // Iterate over columns
            pcl::PointXYZ point = cloud->at(j, i);  // Access point using 2D indexing

            float distance = distanceFromOrigin(point);

            if (distance < min_distance || distance > max_distance) {
                cloud_noise->points.push_back(point);  // Classify as noise
            } else {
                cloud_filtered->points.push_back(point);  // Retain for ground detection
            }
        }
    }

    if (pointCloudCounter == 1 || pointCloudCounter % ransacFrequency == 1) {
        ROS_INFO("Computing RANSAC plane.");
        computeRansacPlane(cloud_filtered);
    }

    // Classify points using the cached RANSAC plane
    classifyUsingCachedPlane(cloud_filtered, cloud_ground, cloud_non_ground);

    // Convert the PCL clouds back to ROS messages and publish them
    sensor_msgs::PointCloud2 output_filtered, output_ground, output_noise;
    pcl::toROSMsg(*cloud_non_ground, output_filtered);
    pcl::toROSMsg(*cloud_ground, output_ground);
    pcl::toROSMsg(*cloud_noise, output_noise);

    output_filtered.header = input->header;  // Preserve original header
    output_ground.header = input->header;
    output_noise.header = input->header;

    pub_filtered.publish(output_filtered);  // Publish non-ground points
    pub_ground.publish(output_ground);      // Publish ground points
    pub_noise.publish(output_noise);        // Publish noise points

    // Calculate and log the processing time
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> processing_time = end_time - start_time;
    ROS_INFO("Processing time: %f seconds", processing_time.count());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_filter");
    ros::NodeHandle nh;

    // Subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/input_point_cloud", 1, pointCloudCallback);

    // Publishers for filtered (non-ground), ground, and noise point clouds
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/ground_point_cloud", 1);
    pub_noise = nh.advertise<sensor_msgs::PointCloud2>("/noise_point_cloud", 1);

    ros::spin();  // Spin the ROS node

    return 0;
}
