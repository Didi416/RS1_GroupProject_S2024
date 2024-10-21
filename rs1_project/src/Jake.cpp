#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "std_msgs/msg/empty.hpp"
#include <vector>
 
class LaserScan : public rclcpp::Node
{
public:
    LaserScan() : Node("laser_scan")
    {
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScan::laserCallback, this, std::placeholders::_1));

        
        
    }
private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        n_ = 'j';
        std::cout << "input a command" << std::endl;
        std::cin >> n_;

        if (n_ == 'j') {
            std::cout << "n = j, no command given" << std::endl;
        }

        
        
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    sensor_msgs::msg::LaserScan laserScan_;
    char n_;

};
 
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
 
    return 0;
}