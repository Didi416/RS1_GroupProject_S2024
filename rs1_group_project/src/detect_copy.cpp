#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/empty.hpp"
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <geometry_msgs/msg/point.hpp>

class CameraBinAndDoorwayDetection : public rclcpp::Node
{
public:
    CameraBinAndDoorwayDetection() : Node("camera_bin_and_doorway_detection")
    {
        // Subscriber to the camera image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&CameraBinAndDoorwayDetection::imageCallback, this, std::placeholders::_1));

        // depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/camera/depth/points", 10, std::bind(&CameraBinAndDoorwayDetection::depthCallback, this, std::placeholders::_1));
        
        goalPub_ = this->create_publisher<geometry_msgs::msg::Point>("/goal_point", 1);
        laserPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_camera_range", 1);

        laser_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CameraBinAndDoorwayDetection::laserCallback, this, std::placeholders::_1));


    }

private:
    // Callback for image processing
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;

            // Detect bins (e.g., green bins)
            detectBins(img);

            // Detect doorways using edge detection
            //detectDoorways(img);

            // Display the processed image with detected bins and doorways
            cv::imshow("Detected Bins and Doorways", img);
            cv::waitKey(1);  // Needed for OpenCV window updates
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    //     if (msg->width == 0 || msg->height == 0) {
    //         RCLCPP_WARN(this->get_logger(), "Received an empty point cloud.");
    //         return;
    //     }
    //     RCLCPP_INFO(this->get_logger(), "check3 ");
    //     float closest_z;
    //     float min_distance = std::numeric_limits<float>::max();
    //     float search_window_ = 0.1;
    //     // Use PointCloud2 iterators to loop through the point cloud
    //     sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    //     sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    //     sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    //     // Define the bounding box around the target X, Y
    //     float x_min = x_point - search_window_;
    //     float x_max = x_point + search_window_;
    //     float y_min = y_point - search_window_;
    //     float y_max = y_point + search_window_;
    //     for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z) {
    //         float x = *iter_x;
    //         float y = *iter_y;
    //         float z = *iter_z;
    //         RCLCPP_INFO(this->get_logger(), "check4 ");
    //         // Check if the point is within the (target area)
    //         if (x >= x_min && x <= x_max && y >= y_min && y <= y_max) {
    //             // Calculate distance to the target (X, Y)
    //             float distance = sqrt(pow(x - x_point, 2) + pow(y - y_point, 2));
    //             // Store the Z value of the closest point in the region
    //             if (distance < min_distance) {
    //                 min_distance = distance;
    //                 closest_z = z;
    //             }
    //         }
    //     }
    //     RCLCPP_INFO(this->get_logger(), "check5 ");
    //     // std::tuple detected_object_point = std::make_tuple(x_point, y_point, closest_z);
    //     send_goal(x_point, y_point, closest_z);
    // }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        // Set range of FOV (camera is 62 degrees directly in front of TB3 so 31 degrees either side of 0)
        int left_index = -31;
        int right_index = 31;

        // Create a new LaserScan message for the filtered data
        auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

        // Clear ranges and intensities
        filtered_scan->ranges.clear();
        filtered_scan->intensities.clear();

        // Only push_back the points within the specified range
        for (int i = left_index; i <= 0; i++) {
            filtered_scan->ranges.push_back(msg->ranges[360+i]); 
            filtered_scan->intensities.push_back(msg->intensities[i]);
            // RCLCPP_INFO(this->get_logger(), "push back: %d", i);
        }
        for (int i = 0; i <= right_index; i++) {
            filtered_scan->ranges.push_back(msg->ranges[i]);
            filtered_scan->intensities.push_back(msg->intensities[i]);
            // RCLCPP_INFO(this->get_logger(), "push back: %d", i);
        }

        // Adjust the angle_min and angle_max to reflect the filtered range
        filtered_scan->angle_min = msg->angle_min + left_index * msg->angle_increment;
        filtered_scan->angle_max = msg->angle_min + right_index * msg->angle_increment;
        // RCLCPP_INFO(this->get_logger(), "Left: %d", left_index);
        // RCLCPP_INFO(this->get_logger(), "Right: %d", right_index);

        laserPub_->publish(*filtered_scan);

        if (objectDetected){
            float object_angle = 0;
            float object_distance = 0;

            // Calculate the index for the laser scan data
            int index = static_cast<int>((msg->angle_min + (x_point * msg->angle_increment)) / msg->angle_increment);

            // Get the distance at that index
            if (index >= 0 && index < msg->ranges.size()) {
                object_distance = msg->ranges[index];
                object_angle = msg->angle_min + index * msg->angle_increment; // Angle to the bin
            }

            float x = object_distance * cos(object_angle);
            float y = object_distance * sin(object_angle);
            geometry_msgs::msg::Point goal_point;
            goal_point.x = x;
            goal_point.y = y;
            goalPub_->publish(goal_point);
        }
    }

    // void detectDoorways(cv::Mat& img) {
    //     // Convert image to grayscale for easier processing
    //     cv::Mat gray;
    //     cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    //     cv::Mat blurred;
    //     cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    //     // Apply Canny edge detection
    //     cv::Mat edges;
    //     cv::Canny(blurred, edges, 50, 150);
    //     // Find contours in the edge-detected image
    //     std::vector<std::vector<cv::Point>> contours;
    //     cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    //     // Filter contours based on the size and shape (we are looking for horizontal gaps)
    //     for (const auto & contour : contours)
    //     {
    //         // Get bounding box of the contour
    //         cv::Rect bounding_box = cv::boundingRect(contour);
    //         // Define conditions for considering this a gap (e.g., width > height and large enough)
    //         if (bounding_box.width > bounding_box.height && bounding_box.width > 100 && bounding_box.height < 50)
    //         {
    //             // Draw a rectangle around the detected gap
    //             cv::rectangle(img, bounding_box, cv::Scalar(0, 255, 0), 2);
    //             // Optionally print or log the location of the gap
    //             // RCLCPP_INFO(this->get_logger(), "Gap detected at (x: %d, y: %d), width: %d, height: %d",
    //                         // bounding_box.x, bounding_box.y, bounding_box.width, bounding_box.height);
    //         }
    //     }
    //     // Display the processed image with detected gaps
    //    // cv::imshow("Detected Gaps", img);
    //     cv::waitKey(1);  // Needed for OpenCV window updates
    // }

    // Detect bins using edge detection and line detection
    void detectBins(cv::Mat& img){
        // Convert the image to HSV color space
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        // Define the color range for the green bin (try a wider range)
        cv::Scalar lower_green(55, 150, 0);   // Adjusted lower bounds for Hue, Saturation, and Value
        cv::Scalar upper_green(65, 255, 50);  // Adjusted upper bounds for Hue, Saturation, and Value

        // Create a mask for green colors
        cv::Mat mask;
        cv::inRange(hsv, lower_green, upper_green, mask);

        cv::imshow("Green Mask", mask);
        cv::waitKey(1);

        // Find contours in the green mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        unsigned int numSize=0;
        x_total = 0; 
        y_total = 0;
        // RCLCPP_INFO(this->get_logger(), "check1");

        for(long unsigned int i=0; i<contours.size(); i++){
            for(long unsigned int j=0; j<contours.at(i).size(); j++){
                x_total += contours.at(i).at(j).x;
                y_total += contours.at(i).at(j).y;
                numSize++;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "check2 ");
        x_point = x_total/numSize;
        y_point = y_total/numSize;

        // Draw outlines around detected green areas (bins)
        for (size_t i = 0; i < contours.size(); i++){
            // Draw the contour outline on the original image
            cv::drawContours(img, contours, (int)i, cv::Scalar(255, 0, 0), 2); // Red color with thickness 2
            objectDetected = true;
            // RCLCPP_INFO(this->get_logger(), "Bin contour detected, index: %zu", i);
        }
    }

    // Subscriber to the camera image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan;
    // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_; 
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goalPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserPub_;
    // std::tuple<float, float, float> detected_object_point;
    float x_total, y_total, x_point, y_point;
    bool objectDetected = false;    
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraBinAndDoorwayDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}