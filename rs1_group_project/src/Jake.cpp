#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/empty.hpp"
#include <vector>
 
class LaserScan : public rclcpp::Node
{
public:
    LaserScan() : Node("laser_scan")
    {
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScan::laserCallback, this, std::placeholders::_1));
        move_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        forward.linear.x = 1;
        backward.linear.x = -1;
        
        
    }
private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::cout << "input a command" << std::endl;
        std::cin >> n_;

        if (n_ == 'j') {
            std::cout << "n = j, no command given" << std::endl;
        }
        else if (n_ == 'w') {
            std::cout << "n = w, move forward" << std::endl;
            move_pub->publish(forward);
        }
        else if (n_ == 's') {
            std::cout << "n = s, move backward" << std::endl;
            move_pub->publish(backward);
        }

        
        
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    sensor_msgs::msg::LaserScan laserScan_;
     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_pub;
    char n_;
    geometry_msgs::msg::Twist forward;
    geometry_msgs::msg::Twist backward;

};
 
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
 
    return 0;
}