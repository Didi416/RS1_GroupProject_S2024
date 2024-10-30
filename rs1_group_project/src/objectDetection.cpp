#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "std_msgs/msg/char.hpp"

class CameraBinAndDoorwayDetection : public rclcpp::Node
{
public:
    CameraBinAndDoorwayDetection() : Node("camera_bin_and_doorway_detection")
    {
        // Subscriber to the camera image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&CameraBinAndDoorwayDetection::imageCallback, this, std::placeholders::_1));
        
        goalPub_ = this->create_publisher<geometry_msgs::msg::Point>("/goal_point", 1);
        laserPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_camera_range", 1);
        markerPub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualisation_marker", 1);

        laser_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CameraBinAndDoorwayDetection::laserCallback, this, std::placeholders::_1));

        odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CameraBinAndDoorwayDetection::odoCallback, this, std::placeholders::_1));

        flag_sub = this->create_subscription<std_msgs::msg::Char>("/flag", 10, std::bind(&CameraBinAndDoorwayDetection::flagCallback, this, std::placeholders::_1));
    }

private:
    // Callback for image processing
    void odoCallback (const nav_msgs::msg::Odometry::SharedPtr msg){
        pose_ = msg->pose.pose;
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
        rclcpp::sleep_for(std::chrono::seconds(2));
        try
        {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;
            half_point = msg->width/2;

            // Detect bins (e.g., green bins)
            if(object_to_detect == 'b'){
                detectBins(img);
            }

            // Detect doorways using edge detection
            if(object_to_detect == 'q'){
                detectDoorways(img);
            }

            // Display the processed image with detected bins and doorways
            cv::imshow("Detected Bins and Doorways", img);
            cv::waitKey(1);  // Needed for OpenCV window updates
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void flagCallback(const std_msgs::msg::Char::SharedPtr msg) {
        object_to_detect = msg->data;
    }

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
        

        if (objectDetected && !goal_published){
            float object_angle = 0;
            float object_distance = 0;

            // Calculate the index for the laser scan data
            int index = static_cast<int>((msg->angle_min + (x_point * msg->angle_increment)) / msg->angle_increment);
            // RCLCPP_INFO(this->get_logger(), "GOAL x_point: %.2f,", x_point);

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

            geometry_msgs::msg::Point transformedPoint = transformPoint(goal_point);
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; //set markers in global frame
            marker.header.stamp = this->now();
            marker.ns = "object";
            marker.id = 1;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = transformedPoint;
            marker.scale.x = 0.5;  // Diameter of the cylinder
            marker.scale.y = 0.5;  // Diameter of the cylinder
            marker.scale.z = 0.1;  // Height of the cylinder
            marker.color.r = 0.0;  // green color
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;     
            markerPub_->publish(marker);
            goalPub_->publish(transformedPoint);
            goal_published = true;
            // RCLCPP_INFO(this->get_logger(), "GOAL PUBLISHED: %.2f, %.2f", goal_point.x, goal_point.y);
            RCLCPP_INFO(this->get_logger(), "GOAL PUBLISHED: %.2f, %.2f", transformedPoint.x, transformedPoint.y);
        }
    }

    //OUTLINE METHOD
//   void detectDoorways(cv::Mat& img) {
//     // Convert the image to grayscale for easier processing
//     cv::Mat gray;
//     cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
//     // Apply Gaussian Blur to reduce noise
//     cv::Mat blurred;
//     cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
//     // Apply Canny edge detection
//     cv::Mat edges;
//     cv::Canny(blurred, edges, 50, 150);
//     // Use Hough Line Transform to detect potential doorway edges
//     std::vector<cv::Vec4i> lines;
//     cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 100, 50, 10);
//     // Store vertical lines that could be potential doorway edges
//     std::vector<cv::Vec4i> verticalLines;
//     // Filter and store vertical lines based on x-coordinates
//     for (const auto& line : lines) {
//         int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
//         // Consider line to be vertical if it is mostly straight up-down
//         if (abs(x1 - x2) < 10) {  // Adjust to fit doorway height
//             verticalLines.push_back(line);
//         }
//     }
//     double pixelToMeterRatio = 0.005;
//     // Convert desired range from meters to pixels using pixelToMeterRatio
//     int minPixelWidth = static_cast<int>(1.0 / pixelToMeterRatio);  // 1 meter
//     int maxPixelWidth = static_cast<int>(1.5 / pixelToMeterRatio);   // 2 meters
//     // Pair vertical lines to outline a doorway without the bottom line
//     for (size_t i = 0; i < verticalLines.size(); ++i) {
//         for (size_t j = i + 1; j < verticalLines.size(); ++j) {
//             int x1_1 = verticalLines[i][0];
//             int x1_2 = verticalLines[j][0];
//             // Check if the two lines are close enough horizontally to be the sides of a doorway
//             int widthInPixels = abs(x1_1 - x1_2);
//             if (widthInPixels > minPixelWidth && widthInPixels < maxPixelWidth) {
//                 // Determine which is the left and right line based on x-coordinates
//                 cv::Vec4i leftLine = (x1_1 < x1_2) ? verticalLines[i] : verticalLines[j];
//                 cv::Vec4i rightLine = (x1_1 < x1_2) ? verticalLines[j] : verticalLines[i];
//                 // Draw the left and right vertical lines in different colors
//                 cv::line(img, cv::Point(leftLine[0], leftLine[1]), 
//                          cv::Point(leftLine[2], leftLine[3]), cv::Scalar(0, 0, 255), 2);  // Red for left line
//                 cv::line(img, cv::Point(rightLine[0], rightLine[1]), 
//                          cv::Point(rightLine[2], rightLine[3]), cv::Scalar(255, 105, 180), 2);  // Pink for right line
//                 // Determine the topmost points of the left and right lines
//                 int topYLeft = std::min(leftLine[1], leftLine[3]);
//                 int topYRight = std::min(rightLine[1], rightLine[3]);
//                 // Use the higher of the two top points to draw the top line
//                 int topY = std::min(topYLeft, topYRight);
//                 // Draw a horizontal top line between the left and right vertical lines at the top level
//                 cv::line(img, cv::Point(leftLine[0], topY), 
//                          cv::Point(rightLine[0], topY), cv::Scalar(0, 255, 0), 2);  // Green for top line
//                 // Print out the coordinates of the lines and which side they are on
//                 std::cout << "Left Line (Red): (" << leftLine[0] << ", " << leftLine[1] << ") to (" 
//                           << leftLine[2] << ", " << leftLine[3] << ")\n";
//                 std::cout << "Right Line (Pink): (" << rightLine[0] << ", " << rightLine[1] << ") to (" 
//                           << rightLine[2] << ", " << rightLine[3] << ")\n";
//                 std::cout << "Top Line (Green): (" << leftLine[0] << ", " << topY << ") to (" 
//                           << rightLine[0] << ", " << topY << ")\n";
//             }
//         }
//     }
//     // Display the processed image with detected doorway edges
//     cv::imshow("Detected Doorway Edges", img);
//     cv::waitKey(1);  // Needed for OpenCV window updates
// }

    //GAP METHOD
    void detectDoorways(cv::Mat& img) {
        // Convert the image to grayscale for easier processing
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian Blur to reduce noise
        cv::Mat blurred;
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

        // Apply Canny edge detection
        cv::Mat edges;
        cv::Canny(blurred, edges, 50, 150);

        // Define the region of interest (ROI) - lower part of the image
        int roiHeight = img.rows / 2;  // Adjust based on how much floor is visible
        cv::Rect roi(0, img.rows - roiHeight, img.cols, roiHeight);
        cv::Mat roiEdges = edges(roi);

        // Find contours in the ROI
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roiEdges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Filter contours based on size and shape (detect horizontal gaps)
        for (const auto& contour : contours)
        {
            // Get bounding box of the contour
            cv::Rect bounding_box = cv::boundingRect(contour);

            // Check if the bounding box meets conditions for a horizontal gap
            if ( bounding_box.width > 350 && bounding_box.width < 600 || bounding_box.width > 650 && bounding_box.width < 1000)
            {
                // Draw a rectangle around the detected gap (shift to original image coordinates)
                cv::rectangle(img, cv::Point(bounding_box.x, bounding_box.y + img.rows - roiHeight),
                            cv::Point(bounding_box.x + bounding_box.width, bounding_box.y + bounding_box.height + img.rows - roiHeight),
                            cv::Scalar(0, 255, 0), 2);

                // Optionally print or log the location of the gap
                RCLCPP_INFO(this->get_logger(), "Gap detected at (x: %d, y: %d), width: %d, height: %d",
                            bounding_box.x, bounding_box.y + img.rows - roiHeight, bounding_box.width, bounding_box.height);
            }
        }

        // Display the processed image with detected gaps
        //cv::imshow("Detected Floor Gaps", img);
        cv::waitKey(1);  // Needed for OpenCV window updates
    }

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

        if(contours.size() == 0){
            objectDetected = false;
        }

        // Draw outlines around detected green areas (bins)
        for (size_t i = 0; i < contours.size(); i++){
            // Draw the contour outline on the original image
            cv::drawContours(img, contours, (int)i, cv::Scalar(255, 0, 0), 2); // Red color with thickness 2
            objectDetected = true;
            // RCLCPP_INFO(this->get_logger(), "Bin contour detected, index: %zu", i);
        }

        if(objectDetected){
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
            x_point = x_total/numSize;
            y_point = y_total/numSize;
            // RCLCPP_INFO(this->get_logger(), "x_point: %.2f", x_point);
        }
    }

    geometry_msgs::msg::Point transformPoint(geometry_msgs::msg::Point point){
        // std::unique_lock<std::mutex> lck(odoMtx_); //lock data access to odometry data (pose_)
        geometry_msgs::msg::Point transformedPoint;
        //transfer position and orientation data to tf2 data type format
        tf2::Vector3 audiPoint(pose_.position.x, pose_.position.y, pose_.position.z); //
        tf2::Quaternion q(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
        // lck.unlock(); //unlock data access for other threads

        tf2::Transform transform(q,audiPoint); //calculate transform of car pose in global frame
        tf2::Vector3 conePoint(point.x, point.y, point.z); //create tf2 data type point to be transformed
        tf2::Vector3 transformedVecPoint = transform * conePoint; //apply transform to point
        //assign values of x,y,z to geometry_msgs/msg/Point
        transformedPoint.x = transformedVecPoint.x();
        transformedPoint.y = transformedVecPoint.y();
        transformedPoint.z = transformedVecPoint.z();
        // RCLCPP_INFO(this->get_logger(), "GOAL PUBLISHED: %.2f, %.2f", transformedPoint.x, transformedPoint.y);

        return transformedPoint; //return transformed point
    }

    // Subscriber to the camera image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr flag_sub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goalPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserPub_;
    geometry_msgs::msg::Pose pose_;
    float x_total, y_total, x_point, y_point;
    bool objectDetected = false;
    bool goal_published = false;
    int half_point;  
    char object_to_detect;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraBinAndDoorwayDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}