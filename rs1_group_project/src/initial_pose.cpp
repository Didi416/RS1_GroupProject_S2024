#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher()
        : Node("initial_pose_publisher")
    {
        // Create publisher for initial pose
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // Timer to publish the initial pose once at startup
        timer_ = this->create_wall_timer(1s, std::bind(&InitialPosePublisher::publish_initial_pose, this));
    }

private:
    void publish_initial_pose()
    {
        // Set initial pose
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = this->get_clock()->now();
        initial_pose.header.frame_id = "map";

        // Position and orientation
        initial_pose.pose.pose.position.x = -2.0;  // Set X position
        initial_pose.pose.pose.position.y = 2.0;  // Set Y position
        initial_pose.pose.pose.position.z = 0.0;
        initial_pose.pose.pose.orientation.z = 0.0;  // Orientation
        initial_pose.pose.pose.orientation.w = 1.0;

        // Publish the pose
        initial_pose_pub_->publish(initial_pose);
        RCLCPP_INFO(this->get_logger(), "Published initial pose to /initialpose");

        // Shutdown the node after publishing once
        rclcpp::shutdown();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
