//E-Stop

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono_literals; // Add this to use ms literal

class EStopNode : public rclcpp::Node
{
public:
    EStopNode() : Node("estop_node"), estop_active_(false)
    {
        // Publisher for velocity commands to stop the TurtleBot
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Timer to continuously publish zero velocity if the e-stop is active
        timer_ = this->create_wall_timer(
            100ms, std::bind(&EStopNode::publish_stop, this));

        // Start a separate thread for capturing user input
        user_input_thread_ = std::thread(&EStopNode::capture_user_input, this);
    }

    ~EStopNode()
    {
        // Ensure the input thread is safely joined on node destruction
        if (user_input_thread_.joinable())
        {
            user_input_thread_.join();
        }
    }

private:
    // Function to capture user input and trigger e-stop
    void capture_user_input()
    {
        std::string input;
        while (rclcpp::ok())
        {
            std::cout << "Type 's' to activate the e-stop, or 'r' to deactivate: ";
            std::cin >> input;

            if (input == "s")
            {
                estop_active_ = true;
                RCLCPP_INFO(this->get_logger(), "E-Stop activated. Stopping the robot.");
            }
            else if (input == "r")
            {
                estop_active_ = false;
                RCLCPP_INFO(this->get_logger(), "E-Stop deactivated. Resuming normal operation.");
            }
            else
            {
                std::cout << "Invalid input. Please type 'stop' or 'resume'.\n";
            }
        }
    }

    // Function to publish zero velocity if the e-stop is active
    void publish_stop()
    {
        if (estop_active_)
        {
            auto stop_msg = geometry_msgs::msg::Twist();
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_msg);
            //RCLCPP_INFO(this->get_logger(), "publishing stop");
        }
    }

    // Variables
    bool estop_active_; // true if e-stop is active
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; // velocity publisher
    rclcpp::TimerBase::SharedPtr timer_; // timer to regularly check and publish zero velocity
    std::thread user_input_thread_; // thread to capture user input
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EStopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



