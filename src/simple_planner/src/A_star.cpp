#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <algorithm>

// Hash function for std::pair<int, int>
struct pair_hash {
    std::size_t operator()(const std::pair<int, int>& pair) const {
        // A simple hashing function for a pair of integers
        return std::hash<int>()(pair.first) ^ std::hash<int>()(pair.second << 1);
    }
};

// Global variables to store the map, costmap, goal, and robot position
nav_msgs::OccupancyGrid::ConstPtr grid_map_;
nav_msgs::OccupancyGrid::ConstPtr costmap_;
geometry_msgs::PoseStamped goal_pose_;
geometry_msgs::Pose robot_pose_;
nav_msgs::Path last_path_msg;  // Store the last computed path

const int OBSTACLE_THRESHOLD = 50;  // Any value greater than this is considered an obstacle
bool goal_received = false;
bool new_goal = false;  // Flag to indicate a new goal has been received
bool map_received = false;
bool costmap_received = false;
bool initial_pose_received = false;

// A* Node structure
struct Node {
    int x, y;
    double g_cost, f_cost;
    int parent_x, parent_y;

    // Default constructor
    Node() : x(0), y(0), g_cost(0.0), f_cost(0.0), parent_x(-1), parent_y(-1) {}

    Node(int x_, int y_, double g_, double f_, int px = -1, int py = -1)
        : x(x_), y(y_), g_cost(g_), f_cost(f_), parent_x(px), parent_y(py) {}

    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }

    bool operator<(const Node& other) const {
        return f_cost < other.f_cost;
    }
};

// Function to calculate Euclidean distance (heuristic)
double calculateHeuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

// Function to get the cost from the costmap and occupancy grid
double getCost(int x, int y, int map_width, int map_height) {
    if (x < 0 || y < 0 || x >= map_width || y >= map_height) {
        return std::numeric_limits<double>::infinity();  // Impassable
    }

    int index = x + y * map_width;

    // Check occupancy grid
    int occupancy_value = grid_map_->data[index];
    if (occupancy_value > OBSTACLE_THRESHOLD) {
        return std::numeric_limits<double>::infinity(); // Occupied cell
    }
    if (occupancy_value == -1) {
        return std::numeric_limits<double>::infinity(); // Unknown cell
    }
    // Get cost from costmap (if available)
    if (costmap_) {
        int cost_value = costmap_->data[index];
        if (cost_value < 0) {
            return std::numeric_limits<double>::infinity();  // Unknown cost
        }
        return cost_value;///255;  // Normalize the cost 
    } else {
        // If costmap is not available, assume uniform cost
        return 0.0;
    }
}

// Reconstruct the path from the end node
std::vector<std::pair<int, int>> reconstructPath(Node& node,
    std::unordered_map<int, std::unordered_map<int, Node>>& all_nodes,
    int start_x, int start_y) {

    std::vector<std::pair<int, int>> path;
    Node current_node = node;

    while (!(current_node.x == start_x && current_node.y == start_y)) {
        path.emplace_back(current_node.x, current_node.y);
        int px = current_node.parent_x;
        int py = current_node.parent_y;
        current_node = all_nodes[px][py];
    }
    path.emplace_back(start_x, start_y);  // Add the start node
    std::reverse(path.begin(), path.end());
    return path;
}

// A* search algorithm
std::vector<std::pair<int, int>> aStarSearch(
    int start_x, int start_y, int goal_x, int goal_y, int map_width, int map_height) {

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    std::unordered_set<std::pair<int, int>, pair_hash> closed_list;
    std::unordered_map<int, std::unordered_map<int, Node>> all_nodes;

    // Initialize start node
    double h_cost = calculateHeuristic(start_x, start_y, goal_x, goal_y);
    Node start_node(start_x, start_y, 0.0, h_cost);
    open_list.push(start_node);
    all_nodes[start_x][start_y] = start_node;

    while (!open_list.empty()) {
        Node current_node = open_list.top();
        open_list.pop();

        // Check if we reached the goal
        if (current_node.x == goal_x && current_node.y == goal_y) {
            return reconstructPath(current_node, all_nodes, start_x, start_y);
        }

        closed_list.insert({current_node.x, current_node.y});

        // Expand neighbors (4-connected grid)
        std::vector<std::pair<int, int>> neighbors = {
            {current_node.x + 1, current_node.y},
            {current_node.x - 1, current_node.y},
            {current_node.x, current_node.y + 1},
            {current_node.x, current_node.y - 1}
        };

        for (const auto& neighbor : neighbors) {
            int nx = neighbor.first;
            int ny = neighbor.second;

            if (nx < 0 || ny < 0 || nx >= map_width || ny >= map_height) {
                continue;  // Skip out of bounds
            }

            // Check if the cell is an obstacle in the occupancy grid
            int map_index = nx + ny * map_width;
            int occupancy_value = grid_map_->data[map_index];

            if (occupancy_value > OBSTACLE_THRESHOLD) {
                continue; // Skip occupied cells (walls)
            }

            if (occupancy_value == -1) {
                // Optionally, treat unknown cells as obstacles
                continue; // Skip unknown cells
            }

            if (closed_list.find({nx, ny}) != closed_list.end()) {
                continue;  // Skip already visited nodes
            }

            double move_cost = 1.0;  // Assume uniform cost for movement
            double costmap_cost = getCost(nx, ny, map_width, map_height);

            if (costmap_cost == std::numeric_limits<double>::infinity()) {
                continue;  // Skip impassable terrain
            }

            double g_cost = current_node.g_cost + move_cost + costmap_cost;
            double h_cost = calculateHeuristic(nx, ny, goal_x, goal_y);
            double total_cost = g_cost + h_cost;

            // Check if this path to neighbor is better
            if (all_nodes.find(nx) == all_nodes.end() ||
                all_nodes[nx].find(ny) == all_nodes[nx].end() ||
                g_cost < all_nodes[nx][ny].g_cost) {

                Node neighbor_node(nx, ny, g_cost, total_cost, current_node.x, current_node.y);
                open_list.push(neighbor_node);
                all_nodes[nx][ny] = neighbor_node;
            }
        }
    }

    // Return an empty path if no solution found
    return {};
}

// Callback function to get the map
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    grid_map_ = msg;
    map_received = true;
    ROS_INFO("Map received.");
}

// Callback function to get the costmap
void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    costmap_ = msg;
    costmap_received = true;
    ROS_INFO("Costmap received.");
}

// Callback function to get the goal position
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Use the map resolution and origin to convert world coordinates to grid coordinates
    double resolution = grid_map_->info.resolution;
    double origin_x = grid_map_->info.origin.position.x;
    double origin_y = grid_map_->info.origin.position.y;

    // Extract the world coordinates (in meters) from the goal
    double goal_x_world = msg->pose.position.x;
    double goal_y_world = msg->pose.position.y;

    // Convert to grid coordinates
    int goal_x_grid = static_cast<int>((goal_x_world - origin_x) / resolution);
    int goal_y_grid = static_cast<int>((goal_y_world - origin_y) / resolution);

    // Store the grid coordinates in the global goal_pose_ (for use in A*)
    goal_pose_.pose.position.x = goal_x_grid;
    goal_pose_.pose.position.y = goal_y_grid;
    goal_received = true;
    new_goal = true;  // Indicate a new goal has been received

    ROS_INFO("Goal received at world coordinates x: %f, y: %f, converted to grid coordinates x: %d, y: %d",
             goal_x_world, goal_y_world, goal_x_grid, goal_y_grid);

    int map_index = goal_x_grid + goal_y_grid * grid_map_->info.width;;
    int occupancy_value = grid_map_->data[map_index];
    ROS_INFO("goal grid value is: %d",
             occupancy_value);
    ROS_INFO("goal costmap value is: %f",
             getCost(goal_x_grid, goal_y_grid, grid_map_->info.width, grid_map_->info.height));
    ROS_INFO("goal costmap_ original value is: %d",
             costmap_->data[map_index]);
}

// Callback function to get the initial pose
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // Use the map resolution and origin to convert world coordinates to grid coordinates
    double resolution = grid_map_->info.resolution;
    double origin_x = grid_map_->info.origin.position.x;
    double origin_y = grid_map_->info.origin.position.y;

    // Extract the world coordinates (in meters) from the initial pose
    double robot_x_world = msg->pose.pose.position.x;
    double robot_y_world = msg->pose.pose.position.y;

    // Convert to grid coordinates
    int robot_x_grid = static_cast<int>((robot_x_world - origin_x) / resolution);
    int robot_y_grid = static_cast<int>((robot_y_world - origin_y) / resolution);

    // Store the grid coordinates in the global robot_pose_ (for use in A*)
    robot_pose_.position.x = robot_x_grid;
    robot_pose_.position.y = robot_y_grid;
    robot_pose_.orientation = msg->pose.pose.orientation;  // Keep orientation (quaternion) the same

    if (!initial_pose_received) {
        initial_pose_received = true;
        ROS_INFO("Initial pose received at world coordinates x: %f, y: %f, converted to grid coordinates x: %d, y: %d",
                 robot_x_world, robot_y_world, robot_x_grid, robot_y_grid);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh;

    // Publisher for the computed path
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 1);

    // Subscribe to the map, costmap, goal, and initial pose
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);
    ros::Subscriber costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/costmap", 1, costmapCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalCallback);
    ros::Subscriber initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseCallback);

    ros::Rate rate(10.0);  // Set the rate at which to run the loop (10Hz)

    while (ros::ok()) {
        ros::spinOnce();

        // If we have the map, initial pose, and goal
        if (map_received && initial_pose_received && goal_received) {
            // If a new goal has been received, run the A* algorithm
            if (new_goal) {
                int map_width = grid_map_->info.width;
                int map_height = grid_map_->info.height;

                int start_x = static_cast<int>(robot_pose_.position.x);
                int start_y = static_cast<int>(robot_pose_.position.y);
                int goal_x = static_cast<int>(goal_pose_.pose.position.x);
                int goal_y = static_cast<int>(goal_pose_.pose.position.y);

                std::vector<std::pair<int, int>> path = aStarSearch(start_x, start_y, goal_x, goal_y, map_width, map_height);

                // Convert the path to nav_msgs/Path
                nav_msgs::Path path_msg;
                path_msg.header.stamp = ros::Time::now();
                path_msg.header.frame_id = "map";

                double resolution = grid_map_->info.resolution;
                double origin_x = grid_map_->info.origin.position.x;
                double origin_y = grid_map_->info.origin.position.y;

                for (const auto& point : path) {
                    geometry_msgs::PoseStamped pose;
                    // Convert grid coordinates back to world coordinates
                    pose.pose.position.x = point.first * resolution + origin_x + resolution / 2.0;
                    pose.pose.position.y = point.second * resolution + origin_y + resolution / 2.0;
                    pose.pose.orientation.w = 1.0;  // No orientation
                    path_msg.poses.push_back(pose);
                }

                if (!path_msg.poses.empty()) {
                    last_path_msg = path_msg;  // Store the path for continuous publishing
                    ROS_INFO("Computed new path with %zu poses.", last_path_msg.poses.size());
                } else {
                    ROS_WARN("No path found by A* algorithm.");
                }

                new_goal = false;  // Reset the new goal flag
            }

            // Publish the last computed path continuously
            if (!last_path_msg.poses.empty()) {
                path_pub.publish(last_path_msg);
            }
        }

        rate.sleep();
    }

    return 0;
}
