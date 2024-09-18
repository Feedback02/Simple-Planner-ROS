#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>
#include <limits>

// Global variable to store the received grid map
nav_msgs::OccupancyGrid::ConstPtr grid_map_;
nav_msgs::OccupancyGrid costmap_msg;  // Global variable for the costmap

// Parameters
const int OBSTACLE_THRESHOLD = 50; // Any value greater than this is considered an obstacle

// Cost map to store costs for each cell
std::vector<double> cost_map;

// Function to convert the cost map to an occupancy grid
void convertCostMapToOccupancyGrid() {
    costmap_msg.header.stamp = ros::Time::now();
    costmap_msg.header.frame_id = "map";  // The frame in which the map is located
    costmap_msg.info.width = grid_map_->info.width;
    costmap_msg.info.height = grid_map_->info.height;
    costmap_msg.info.resolution = grid_map_->info.resolution;
    costmap_msg.info.origin = grid_map_->info.origin;

    // Convert the cost map to occupancy grid format (0-100)
    costmap_msg.data.resize(cost_map.size(), 0);
    for (size_t i = 0; i < cost_map.size(); ++i) {
        if (cost_map[i] == std::numeric_limits<double>::infinity()) {
            costmap_msg.data[i] = 100;  // Mark obstacles as 100
        } else {
            costmap_msg.data[i] = static_cast<int8_t>(std::min(100.0, cost_map[i] * 100.0));  // Normalize to 0-100 scale
        }
    }
}

// Callback function to process the grid map
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    grid_map_ = msg;

    // Initialize the cost map to the same size as the occupancy grid
    int width = grid_map_->info.width;
    int height = grid_map_->info.height;
    cost_map.resize(width * height, 0.0);

    // Compute the cost for each cell based on its distance to the nearest obstacle
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = x + y * width;
            int cell_value = grid_map_->data[index];

            if (cell_value > OBSTACLE_THRESHOLD) {
                // This is an obstacle, assign a high cost
                cost_map[index] = std::numeric_limits<double>::infinity();
            } else {
                // Compute the cost based on the distance to the nearest obstacle
                double min_distance = std::numeric_limits<double>::infinity();

                // Scan through the map to find the closest obstacle
                for (int oy = 0; oy < height; ++oy) {
                    for (int ox = 0; ox < width; ++ox) {
                        int o_index = ox + oy * width;
                        if (grid_map_->data[o_index] > OBSTACLE_THRESHOLD) {
                            // Obstacle found, compute distance to this obstacle
                            double distance = std::sqrt(std::pow(x - ox, 2) + std::pow(y - oy, 2));
                            if (distance < min_distance) {
                                min_distance = distance;
                            }
                        }
                    }
                }

                // Set the cost inversely proportional to the distance (closer = higher cost)
                cost_map[index] = 1.0 / (min_distance + 1.0);  // Adding 1 to avoid division by zero
            }
        }
    }

    ROS_INFO("Cost map generated.");
    
    // After processing the map, convert it to OccupancyGrid format
    convertCostMapToOccupancyGrid();
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "cost_function");
    ros::NodeHandle nh;

    // Publisher to publish the costmap
    ros::Publisher costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 10);

    // Subscribe to the grid map topic ("/map") and process it
    ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);


    // Set up a loop to publish the costmap periodically
    ros::Rate loop_rate(1);  // 1 Hz loop (publish once per second)

    while (ros::ok()) {
        costmap_pub.publish(costmap_msg);  // Publish the pre-processed costmap
        ROS_INFO("Costmap published.");
                
        ros::spinOnce(); // Process incoming messages (e.g., map data)
        loop_rate.sleep();  // Wait for the next loop cycle
    }

    return 0;
}
