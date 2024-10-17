#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "std_msgs/msg/empty.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf2/utils.h> //To use getYaw function from the quaternion of orientation
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include <opencv2/opencv.hpp>
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
        
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    sensor_msgs::msg::LaserScan laserScan_;
    bool found_;
    unsigned int ct_; //!< Marker Count
    int n_;
    int count;
};
 
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
 
    return 0;
}