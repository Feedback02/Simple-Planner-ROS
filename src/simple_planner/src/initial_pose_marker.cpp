#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

class InitialPoseMarker
{
public:
    InitialPoseMarker()
    {
        // Initialize the subscriber to /initialpose
        initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &InitialPoseMarker::initialPoseCallback, this);

        // Initialize the publisher for the marker
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/initial_pose_marker", 1);

        // Initialize the marker
        marker_.header.frame_id = "map";
        marker_.ns = "initial_pose";
        marker_.id = 0;
        marker_.type = visualization_msgs::Marker::SPHERE;  // Changed to SPHERE
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.scale.x = 0.3;  // Diameter in x
        marker_.scale.y = 0.3;  // Diameter in y
        marker_.scale.z = 0.3;  // Diameter in z
        marker_.color.a = 1.0;  // Alpha
        marker_.color.r = 0.0;  // Red
        marker_.color.g = 1.0;  // Green
        marker_.color.b = 0.0;  // Blue
        marker_.lifetime = ros::Duration(0);  // Make marker persistent
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        ROS_INFO("Received initial pose message.");

        // Update marker's header
        marker_.header.stamp = ros::Time::now();

        // Set the position of the marker to the initial pose position
        marker_.pose.position = msg->pose.pose.position;
        marker_.pose.orientation.w = 1.0;  // Orientation is irrelevant for a sphere

        // Publish the marker
        marker_pub_.publish(marker_);

        ROS_INFO("Initial pose sphere marker updated at position x: %f, y: %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber initial_pose_sub_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker marker_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial_pose_marker");
    InitialPoseMarker initial_pose_marker;

    ros::spin();

    return 0;
}
