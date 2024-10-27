#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/empty.hpp"
#include <vector>
#include <thread>
#include <chrono>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals; // Add this to use ms literal
 
class LaserScan : public rclcpp::Node
{
public:
    LaserScan() : Node("laser_scan")
    {
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScan::laserCallback, this, std::placeholders::_1));
        img_sub = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10, std::bind(&LaserScan::imageCallback, this, std::placeholders::_1));
        move_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        // Create a client for managing lifecycle nodes
        lifecycle_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");

        user_input_thread_ = std::thread(&LaserScan::move, this);

        forward.linear.x = 0.5;
        backward.linear.x = -0.5;
        left.angular.z = 0.5;
        right.angular.z = -0.5;
        stop.linear.x = 0;
        safe = true;
        stopped = false;
        estopped = false;
        
    }
    ~LaserScan()
    {
        // Ensure the input thread is safely joined on node destruction
        if (user_input_thread_.joinable())
        {
            user_input_thread_.join();
        }
    }
private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        for(int x = 0; x < 20; x++) {
            if(std::isfinite(msg->ranges.at(x)) && !std::isnan(msg->ranges.at(x))) {
                //std::cout << x << "beans" << msg->ranges.at(x) << std::endl;
                if (msg->ranges[x] <= 1) {
                    safe = false;
                    //std::cout << "beans" << msg->ranges[x] << std::endl;
                    break;
                }
                //std::cout << "beans safe" << std::endl;
                safe = true;
            }
        }
        if (safe) {
            for(int x = 340; x < 360; x++) {
                if(std::isfinite(msg->ranges.at(x)) && !std::isnan(msg->ranges.at(x))) {
                    //std::cout << x << "beans" << msg->ranges.at(x) << std::endl;
                    if (msg->ranges[x] <= 1) {
                        safe = false;
                        //std::cout << "beans" << msg->ranges[x] << std::endl;
                        break;
                    }
                    //std::cout << "beans safe" << std::endl;
                    safe = true;
                    stopped = false;
                }
            }
        }
        if (!safe) {
            if(!stopped) {
                move_pub->publish(stop);
                std::cout << "object detected, unsafe to move forward" << std::endl;
                stopped = true;
            }
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

    }

    void move() {
        while(true) {
            std::cout << "input a command" << std::endl;
            std::cin >> n_;

            if (n_ == 'e') {
                stop_navigation_service();
                std::cout << "n = e, stop now!" << std::endl;
                estopped = !estopped;
                move_pub->publish(stop);
            }
            if (!estopped) {
                stop_navigation_service();
                if (n_ == 'w') {
                    if (safe) {
                        std::cout << "n = w, move forward" << std::endl;
                        move_pub->publish(forward);
                    }
                    else {
                        std::cout << "unsafe to move forwards" << std::endl;
                        move_pub->publish(stop);
                    }
                }
                else if (n_ == 'a') {
                    std::cout << "n = a, move left" << std::endl;
                    move_pub->publish(left);
                }
                else if (n_ == 'd') {
                    std::cout << "n = d, move right" << std::endl;
                    move_pub->publish(right);
                }
            }
        }
    }

    /**
     * @brief Stops the navigation service
     */
    void stop_navigation_service() {
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        RCLCPP_INFO(this->get_logger(), "Stopping navigation service...");
        request->command = 1; // Command to stop the service
        RCLCPP_INFO(this->get_logger(), "Sending command to stop navigation service...");
        //Send the command to stop the navigation service
        auto result_future = lifecycle_client_->async_send_request(request);
 
 
        // Send command to deactivate the service
        request->command = 2; // Command to deactivate the service
        result_future = lifecycle_client_->async_send_request(request);
 
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    sensor_msgs::msg::LaserScan laserScan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_pub;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr lifecycle_client_; // Client for lifecycle service
    //rclcpp::TimerBase::SharedPtr timer_; // timer
    std::thread user_input_thread_; // thread to capture user input
    char n_;
    geometry_msgs::msg::Twist stop;
    geometry_msgs::msg::Twist forward;
    geometry_msgs::msg::Twist backward;
    geometry_msgs::msg::Twist left;
    geometry_msgs::msg::Twist right;
    bool safe;
    bool stopped;
    bool estopped;

};
 
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
 
    return 0;
}