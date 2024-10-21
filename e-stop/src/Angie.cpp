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
            std::cout << "Type 'stop' to activate the e-stop, or 'resume' to deactivate: ";
            std::cin >> input;

            if (input == "stop")
            {
                estop_active_ = true;
                RCLCPP_INFO(this->get_logger(), "E-Stop activated. Stopping the robot.");
            }
            else if (input == "resume")
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




// class eStop : public rclcpp::Node
// {
// public:
//     eStop() : Node("eStop")
//     {
//         laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScan::laserCallback, this, std::placeholders::_1)); 
        
//     }
// private:
//     void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//     {
//         //detects object in front of them
        
//     }

//     void


//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
//     sensor_msgs::msg::LaserScan laserScan_;
//     bool found_;
//     unsigned int ct_; //!< Marker Count
//     int n_;
//     int count;
// }


// // Calculation function for the total distance function
// double robot::calculateDistance(const pfms::geometry_msgs::Point& start, const pfms::geometry_msgs::Point& end) {
//     return std::sqrt(std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2));
// };


// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<LaserScan>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
 
//     return 0;
// }