#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

// Global variable to store the received grid map
nav_msgs::OccupancyGrid::ConstPtr grid_map_;
nav_msgs::OccupancyGrid costmap_msg;  // Global variable for the costmap

// Parameters
const int OBSTACLE_THRESHOLD = 50;  // Any value greater than this is considered an obstacle
double inflation_radius = 0.2;      // Inflation radius in meters, the influence of an obstacle in meters in all directions.
double cost_scaling_factor = 10.0;  // Scaling factor for cost computation

// Callback function to process the grid map
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    grid_map_ = msg;

    int width = grid_map_->info.width;
    int height = grid_map_->info.height;
    double resolution = grid_map_->info.resolution;

    // Convert occupancy grid to binary image
    cv::Mat binary_map(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = x + y * width;
            int cell_value = grid_map_->data[index];
            if (cell_value > OBSTACLE_THRESHOLD) {
                binary_map.at<uchar>(y, x) = 0;  // Obstacle
            } else {
                binary_map.at<uchar>(y, x) = 255;  // Free space
            }
        }
    }

    // Apply distance transform to calculate distances from obstacles
    cv::Mat distance_map;
    cv::distanceTransform(binary_map, distance_map, cv::DIST_L2, 3);

    // Convert inflation radius from meters to grid cells
    double max_distance = inflation_radius / resolution;

    // Normalize the distance map based on the inflation radius
    cv::Mat normalized_distance_map = distance_map / max_distance;
    normalized_distance_map.setTo(1.0, normalized_distance_map > 1.0);  // Cap at 1.0

    // Generate the cost map: cells near obstacles have higher costs
    cv::Mat cost_map_cv = (1.0 - normalized_distance_map) * 99.0;  // Scale to 0-99 (reserving 100 for lethal obstacles)
    cost_map_cv.convertTo(cost_map_cv, CV_8UC1);  // Convert to 8-bit unsigned char

    // Set obstacles and unknowns in the costmap
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = x + y * width;
            int cell_value = grid_map_->data[index];
            if (cell_value > OBSTACLE_THRESHOLD) {
                cost_map_cv.at<uchar>(y, x) = 100;  // Lethal obstacle
            } else if (cell_value == -1) {
                cost_map_cv.at<uchar>(y, x) = 255;  // Unknown
            }
        }
    }

    // Prepare the costmap message
    costmap_msg.header.stamp = ros::Time::now();
    costmap_msg.header.frame_id = "map";  // The frame in which the map is located
    costmap_msg.info.width = width;
    costmap_msg.info.height = height;
    costmap_msg.info.resolution = resolution;
    costmap_msg.info.origin = grid_map_->info.origin;

    // Convert the cost_map_cv to a 1D array for the OccupancyGrid message
    costmap_msg.data.resize(width * height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = x + y * width;
            costmap_msg.data[index] = cost_map_cv.at<uchar>(y, x);
        }
    }

    ROS_INFO("Costmap generated.");
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "costmap_generator");
    ros::NodeHandle nh;

    // Get parameters from the parameter server (if available)
    nh.param("inflation_radius", inflation_radius, 0.2);
    nh.param("cost_scaling_factor", cost_scaling_factor, 10.0);

    // Publisher to publish the costmap
    ros::Publisher costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);

    // Subscribe to the grid map topic ("/map") and process it
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);

    // Set up a loop to publish the costmap periodically
    ros::Rate loop_rate(1);  // 1 Hz loop (publish once per second)

    while (ros::ok()) {
        if (grid_map_) {
            costmap_pub.publish(costmap_msg);  // Publish the generated costmap
            ROS_INFO("Costmap published.");
        }

        ros::spinOnce();  // Process incoming messages (e.g., map data)
        loop_rate.sleep();  // Wait for the next loop cycle
    }

    return 0;
}
