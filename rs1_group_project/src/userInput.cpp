#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "std_msgs/msg/char.hpp"
#include <vector>
#include <thread>
#include <chrono>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals; // Add this to use ms literal
 
class LaserScan : public rclcpp::Node
{
public:
    LaserScan() : Node("laser_scan")
    {
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScan::laserCallback, this, std::placeholders::_1));
        goal_sub = this->create_subscription<geometry_msgs::msg::Point>("/goal_point", 10, std::bind(&LaserScan::goalCallback, this, std::placeholders::_1));
        odo_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LaserScan::odoCallback, this, std::placeholders::_1));
        flag_pub = this->create_publisher<std_msgs::msg::Char>("/flag", 1);
        move_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        // Create a client for managing lifecycle nodes
        lifecycle_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");

        navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(  
            this,   "navigate_to_pose"  ); 

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

    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "GOAL FOUND");
        objectGoal = *msg;
        send_goal(msg->x, msg->y);
    }

    void odoCallback (const nav_msgs::msg::Odometry::SharedPtr msg){
        pose_ = msg->pose.pose;
    }

    void move() {
        while(true) {
            // if(goal_published && pose_.position == objectGoal){
            //     object_to_detect.data = 'z';
            //     flag_pub->publish(object_to_detect);
            // }
            std::cout << "input a command" << std::endl;
            std::cin >> n_;

            if (n_ == "e") {
                stop_navigation_service();
                std::cout << "n = e, stop now!" << std::endl;
                estopped = !estopped;
                move_pub->publish(stop);
            }
            if (!estopped) {
                if (n_ == "w") {
                    if (safe) {
                        stop_navigation_service();
                        std::cout << "n = w, move forward" << std::endl;
                        move_pub->publish(forward);
                    }
                    else {
                        std::cout << "unsafe to move forwards" << std::endl;
                        move_pub->publish(stop);
                    }
                }
                else if (n_ == "a") {
                    stop_navigation_service();
                    std::cout << "n = a, move left" << std::endl;
                    move_pub->publish(left);
                }
                else if (n_ == "d") {
                    stop_navigation_service();
                    std::cout << "n = d, move right" << std::endl;
                    move_pub->publish(right);
                }
                else if (n_ == "b"){
                    object_to_detect.data = 'b';
                    flag_pub->publish(object_to_detect);
                }
                else if (n_ == "q"){
                    object_to_detect.data = 'q';
                    flag_pub->publish(object_to_detect);
                }
                else if(n_ == "1"){
                    std::cout << "Location 1" << std::endl;
                    send_goal(-3.4, 0.3);
                }
                else if(n_ == "2"){
                    std::cout << "Location 2" << std::endl;
                    send_goal(-6.2, 1.6);
                }
                else if(n_ == "3"){
                    std::cout << "Location 3" << std::endl;
                    send_goal(-2.4, 7.0);
                }
                else if(n_ == "4"){
                    std::cout << "Location 4" << std::endl;
                    send_goal(1.1, 6.4);
                }
                else if(n_ == "5"){
                    std::cout << "Location 5" << std::endl;
                    send_goal(5.0, 4.0);
                }
                else if(n_ == "6"){
                    std::cout << "Location 6" << std::endl;
                    send_goal(6.0, 1.7);
                }
                else if(n_ == "7"){
                    std::cout << "Location 7" << std::endl;
                    send_goal(3.9, -3.6);
                }
                else if(n_ == "8"){
                    std::cout << "Location 8" << std::endl;
                    send_goal(1.0, -4.4);
                }
                else if(n_ == "9"){
                    std::cout << "Location 9" << std::endl;
                    send_goal(-2.3, -2.8);
                }
                else if(n_ == "10"){
                    std::cout << "Location 10" << std::endl;
                    send_goal(-5.5, -4.5);
                }
            }
        }
    }


    void send_goal(double x, double y) {
        // Ensure the client is available
        if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }
       RCLCPP_INFO(this->get_logger(), "Sending new goal to the navigation service...");

        // Create goal message
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        // Send the goal with options
        auto end_goal_future = navigate_to_pose_client_->async_send_goal(goal_msg);

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
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr flag_pub;
    sensor_msgs::msg::LaserScan laserScan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_pub;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr lifecycle_client_; // Client for lifecycle service
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_; 
    //rclcpp::TimerBase::SharedPtr timer_; // timer
    std::thread user_input_thread_; // thread to capture user input
    std::string n_;
    geometry_msgs::msg::Twist stop;
    geometry_msgs::msg::Twist forward;
    geometry_msgs::msg::Twist backward;
    geometry_msgs::msg::Twist left;
    geometry_msgs::msg::Twist right;
    geometry_msgs::msg::Pose pose_;
    geometry_msgs::msg::Point objectGoal;
    std_msgs::msg::Char object_to_detect;
    bool goal_published = false;
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