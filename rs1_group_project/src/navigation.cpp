#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/empty.hpp"
#include <vector>

class Navigator : public rclcpp::Node
{
public:
    Navigator() : Node("laser_scan")
    {
        // laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScan::laserCallback, this, std::placeholders::_1));
        // move_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        // forward.linear.x = 1;
        // backward.linear.x = -1;
        
        
    }
private:

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Navigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
 
    return 0;
}