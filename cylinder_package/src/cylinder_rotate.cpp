#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


/**
 * @brief A class to detect a cylinder from a set of points
 */
class CylinderNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Cylinder Node object
     */
    CylinderNode() : Node("cylinder_detect") {
        //odometry_node_ = std::make_shared<OdometryNode>(); 
        // Create a subscriber for the LaserScan message
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderNode::laser_callback, this, std::placeholders::_1));
        // Create a subscriber for the Odometry message
        pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CylinderNode::odom_callback, this, std::placeholders::_1));
        // Create a publisher for the Marker message
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("cylinder_marker", 10);
        // Create a client for managing lifecycle nodes -> Navigation 2 cancellation service
        lifecycle_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            "/lifecycle_manager_navigation/manage_nodes");

        RCLCPP_INFO(this->get_logger(), "Creating action client to navigate to pose");
        // Create a client for the NavigateToPose action (NAV 2 interaction)
        navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>( 
            this,   "navigate_to_pose"  );
        // Sleep as we wait for RVIZ and Gazebo to open   
        rclcpp::sleep_for(std::chrono::seconds(10));
        RCLCPP_INFO(this->get_logger(), "SENDING GOAL");
        // Send the goal to the navigation service
        send_goal();
    }

private:
    // Create the subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr lifecycle_client_; 
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;

    
    // Create the variables
    nav_msgs::msg::Odometry robot_pose_; 
    std::vector<geometry_msgs::msg::Point> points_in_segment; 
    int read_counter = 0;
    int read_counter2 = 0;
    bool detected = false;
    float map_x= 0.0;
    float map_y = 0.0;
    bool all_reached = false;
    int goal_number = 0;
    double robot_start_x = 0.0;
    double robot_start_y = 0.0;
    int count = 0;

    /**
     * @brief Callback function for the Odometry message 
     * @param msg The Odometry message
     */
    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
        robot_pose_ = *msg; // Update the robot's pose
    }

    /**
     * @brief Callback function for the LaserScan message
     * @param msg The LaserScan message
     */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        // Constants for cylinder 
        const float diameter = 0.3;  // Cylinder diameter in meters
        const float radius = diameter / 2.0; 
        const float circumference = M_PI * diameter; 
        const float half_circumference = circumference / 2.0; 
        const float threshold = 0.1; // Tolerance for cylinder detection
        std::vector<geometry_msgs::msg::Point> detected_points;

        // Process the Lidar data to find points that could form a cylinder
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            // Check if the scan value is valid
            // Reduce max scan range to minimise errors
            if (msg->ranges[i] < 2.5 && msg->ranges[i] > msg->range_min) {
                // Convert polar coordinates to Cartesian
                float angle = msg->angle_min + i * msg->angle_increment;
                float x = msg->ranges[i] * std::cos(angle);
                float y = msg->ranges[i] * std::sin(angle);
                // Store the detected point
                geometry_msgs::msg::Point point;
                point.x = x;
                point.y = y;
                point.z = 0.0; 
                // Create a vector of detected points to be checked for cylinder
                detected_points.push_back(point);
            }
        }

        // Check for cylinder in the detected points
        if (detect_cylinder(detected_points, half_circumference, threshold)) {
            // Read counter to reduce error
            read_counter++;
            // Must have 15 correct reads before confirming
            if (read_counter == 15 && !detected) {
                RCLCPP_INFO(this->get_logger(), "Cylinder detected!");
                publish_marker(points_in_segment);  // Publish marker at the detected points
                stop_navigation_service();  // Stop the navigation service
                // sleep for 5 seconds 
                rclcpp::sleep_for(std::chrono::seconds(5));
                read_counter = 1000;
                detected = true;
                // Update global variable for robot's starting position
                robot_start_x = robot_pose_.pose.pose.position.x;
                robot_start_y = robot_pose_.pose.pose.position.y;
            }
            // If the cylinder is detected, send the goal to navigate around the cylinder
            else if (detected) {
                send_cylinder_goal();
                // Check if all goals are reached
                if (all_reached) {
                    RCLCPP_INFO(this->get_logger(), "All goals reached.");
                    // Send the original goal to the navigation service
                    send_goal();
                }
            }
        } else {
            read_counter2++;
            // Must have 2 correct reads before confirming
            if (read_counter2 == 2) {
                RCLCPP_INFO(this->get_logger(), "No cylinder detected.");
                //delete_marker();  // Delete the marker
                // Reset the read counters
                read_counter2 = 0;
                read_counter = 0;
            }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(30));
    }

    /**
     * @brief Stops the navigation service
     */
    void stop_navigation_service() {
        // Create a request to stop the navigation service
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        RCLCPP_INFO(this->get_logger(), "Stopping navigation service...");
        request->command = 1; // Command to stop the service
        RCLCPP_INFO(this->get_logger(), "Sending command to stop navigation service...");
        //Send the command to stop the navigation service
        auto result_future = lifecycle_client_->async_send_request(request);
       // Send command to reactivate the service
        request->command = 2; 
        result_future = lifecycle_client_->async_send_request(request);

    }

    /**
     * @brief Sends the starting goal to the navigation service
     */

    void send_goal() {
        // Ensure the client is available to send goals
        if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }
 
        // Create goal message
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = 4.0;
        goal_msg.pose.pose.position.y = 2.0;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        // Send the goal
        auto end_goal_future = navigate_to_pose_client_->async_send_goal(goal_msg);
    }
    /**
     * @brief Sends the goal to navigate around the cylinder. Similar to send_goal() but with different goals
     */
    void send_cylinder_goal() {
        // Ensure the client is available
        if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // Cylinder radius (0.3) + additional distance (0.6) = 0.9 radius from the center
        double radius = 0.9;
        // Create an array of 4 goal positions (90-degree intervals around the cylinder)
        std::vector<std::pair<double, double>> goals = {
            {map_x, map_y - radius},                // Point 0 (right)
            {map_x + radius, map_y},                // Point 1 (above)
            {map_x, map_y + radius},                // Point 2 (left)
            {map_x - radius, map_y},                // Point 3 (below)
            {map_x, map_y - radius}                 // Point 4 (right)
        };
        // Calculate the distance between the robot's current position and the goal currently being travelled to
        double distance = std::sqrt(std::pow(goals[goal_number-1].first - robot_pose_.pose.pose.position.x, 2) + std::pow(goals[goal_number-1].second -  robot_pose_.pose.pose.position.y, 2));
        // Add the robot's starting position as the last goal
        goals.push_back({robot_start_x, robot_start_y});      
        // Loop through each goal and send the navigation command if close to previous goal or is the starting goal
            if (goal_number == 0 || distance < 0.3) {
                
                auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
                goal_msg.pose.header.frame_id = "map";
                goal_msg.pose.pose.position.x = goals[goal_number].first;
                goal_msg.pose.pose.position.y = goals[goal_number].second;
                goal_msg.pose.pose.position.z = 0.0;
                goal_msg.pose.pose.orientation.x = 0.0;
                goal_msg.pose.pose.orientation.y = 0.0;
                goal_msg.pose.pose.orientation.z = 0.0;
                goal_msg.pose.pose.orientation.w = 1.0;
                goal_number++;
                // Send the goal
                auto end_goal_future = navigate_to_pose_client_->async_send_goal(goal_msg);
                RCLCPP_INFO(this->get_logger(), "Sending goal %d to the navigation service...", goal_number);
                // Check if all goals are reached to change flag
                if (goal_number == 6) {
                    all_reached = true;
                    RCLCPP_INFO(this->get_logger(), "All goals reached.");
                }
            }
        return;
    }

    /**
    * @brief Detects a cylinder from a set of points
    * @param points The set of points
    * @param diameter The diameter of the cylinder
    * @param threshold The tolerance for detection
    * @return true if a cylinder is detected, false otherwise
    */
    bool detect_cylinder(const std::vector<geometry_msgs::msg::Point>& points, float diameter, float threshold) {
        // If the laser scan doesn't pick up anything 
        if (points.empty()) {
            return false; // Handle empty input
        }
        
        geometry_msgs::msg::Point previous_point = points[0];
        double total_distance = 0.0;
        bool isInSegment = false;
        // Loop through the points to check for a cylinder
        for (size_t i = 1; i < points.size(); ++i) {
                const auto& point = points[i];
                double dx = point.x - previous_point.x;
                double dy = point.y - previous_point.y;
                double distance_sq = dx * dx + dy * dy; 

                // If readings are close together, assume they are from the same object
                if (distance_sq < 0.2) { 
                    if (!isInSegment) {
                        isInSegment = true;
                        // Clear the points from previous segment
                        total_distance = 0.0;
                    }
                    total_distance += std::sqrt(distance_sq); // Sum distance of the segment
                    points_in_segment.push_back(point); // Store the point in the segment
                // When the object has ended
                } else {
                    if (isInSegment) {
                        // Check if the segment is a cylinder
                        // smmaler upper tolernce to avoid errors
                        if (total_distance < diameter + (threshold/2) && total_distance > diameter - (threshold * 1.5) ){
                            points_in_segment.push_back(point);
                            return true; // Found a cylinder
                        } else {
                            // Clear the points if not a cylinder
                            points_in_segment.clear();
                            total_distance = 0.0;
                        }
                    }
                    // Reset the segment
                    isInSegment = false;
                }
                previous_point = point; // Update previous_point 
        }
        return false; // No cylinder detected
    }


    /**
    * @brief Publishes a marker at the centroid of the detected points
    * @param points The detected points
    */
    void publish_marker(const std::vector<geometry_msgs::msg::Point>& points) {
        if (points.empty()) {
            return; // No points, no marker to publish
        }

        // Calculate the centroid of the points
        float sum_x = 0.0, sum_y = 0.0;
        for (const auto& point : points) {
            sum_x += point.x;
            sum_y += point.y;
        }

        float centroid_x = sum_x / points.size();
        float centroid_y = sum_y / points.size();

        // Get the robot's position
        double x_position = robot_pose_.pose.pose.position.x;  
        double y_position = robot_pose_.pose.pose.position.y;  

        // Extract the quaternion from robot_pose_
        double qx = robot_pose_.pose.pose.orientation.x;
        double qy = robot_pose_.pose.pose.orientation.y;
        double qz = robot_pose_.pose.pose.orientation.z;
        double qw = robot_pose_.pose.pose.orientation.w;

        // Calculate the yaw angle from the quaternion
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = atan2(siny_cosp, cosy_cosp); // Yaw angle in radians

        // Transform the centroid to the robot frame
        // Rotation transformation
        map_x = x_position + (centroid_x * cos(yaw) - centroid_y * sin(yaw));
        map_y = y_position + (centroid_x * sin(yaw) + centroid_y * cos(yaw));

        // Create a marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Set to the correct frame
        marker.header.stamp = this->now();
        marker.ns = "cylinder_detection";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the marker (use the transformed position)
        marker.pose.position.x = map_x;  
        marker.pose.position.y = map_y;  
        marker.pose.position.z = 0.0; 
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.3;  // Diameter of the cylinder
        marker.scale.y = 0.3;  // Diameter of the cylinder
        marker.scale.z = 0.05;   

        marker.color.r = 1.0;  // Red
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;        

        // Publish the marker
        RCLCPP_INFO(this->get_logger(), "Publishing marker");
        marker_publisher_->publish(marker);
    }

    /**
    * @brief Deletes the marker if wanted
    */
    void delete_marker() {
        // Create a marker with action DELETE
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Set to the correct frame
        marker.header.stamp = this->now();
        marker.ns = "cylinder_detection";
        marker.id = 1;
        marker.action = visualization_msgs::msg::Marker::DELETE;  // Set action to DELETE

        // Publish the deletion marker
        marker_publisher_->publish(marker);
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderNode>());
    rclcpp::shutdown();
    return 0;
}
