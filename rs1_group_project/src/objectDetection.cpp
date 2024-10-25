// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include "std_msgs/msg/empty.hpp"
// #include <vector>
// #include <thread>
// #include <chrono>

// #include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
// #include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
// #include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 

// using namespace std::chrono_literals; // Add this to use ms literal
 
// class LaserScan : public rclcpp::Node
// {
// public:
//     LaserScan() : Node("laser_scan")
//     {
//         laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScan::laserCallback, this, std::placeholders::_1));
//         img_sub = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10, std::bind(&LaserScan::imageCallback, this, std::placeholders::_1));
//         move_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

//         user_input_thread_ = std::thread(&LaserScan::move, this);

//         forward.linear.x = 0.5;
//         backward.linear.x = -0.5;
//         left.angular.z = 0.5;
//         right.angular.z = -0.5;
//         stop.linear.x = 0;
//         safe = true;
//         stopped = false;
//         estopped = false;
        
//     }
//     ~LaserScan()
//     {
//         // Ensure the input thread is safely joined on node destruction
//         if (user_input_thread_.joinable())
//         {
//             user_input_thread_.join();
//         }
//     }

// private:
//     void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//     {

//         for(int x = 0; x < 20; x++) {
//             if(std::isfinite(msg->ranges.at(x)) && !std::isnan(msg->ranges.at(x))) {
//                 //std::cout << x << "beans" << msg->ranges.at(x) << std::endl;
//                 if (msg->ranges[x] <= 1) {
//                     safe = false;
//                     //std::cout << "beans" << msg->ranges[x] << std::endl;
//                     break;
//                 }
//                 //std::cout << "beans safe" << std::endl;
//                 safe = true;
//             }
            
//         }
//         if (safe) {
//             for(int x = 340; x < 360; x++) {
//                 if(std::isfinite(msg->ranges.at(x)) && !std::isnan(msg->ranges.at(x))) {
//                     //std::cout << x << "beans" << msg->ranges.at(x) << std::endl;
//                     if (msg->ranges[x] <= 1) {
//                         safe = false;
//                         //std::cout << "beans" << msg->ranges[x] << std::endl;
//                         break;
//                     }
//                     //std::cout << "beans safe" << std::endl;
//                     safe = true;
//                     stopped = false;
//                 }
//             }
//         }
//         if (!safe) {
//             if(!stopped) {
//                 move_pub->publish(stop);
//                 std::cout << "object detected, unsafe to move forward" << std::endl;
//                 stopped = true;
//             }
            
//         }
        
//     }

//     void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         try
//         {
//             // Convert ROS image message to OpenCV format
//             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             cv::Mat img = cv_ptr->image;

//             // Convert image to grayscale for easier processing
//             cv::Mat gray;
//             cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

//             // Apply Gaussian Blur to reduce noise
//             cv::Mat blurred;
//             cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

//             // Apply Canny edge detection
//             cv::Mat edges;
//             cv::Canny(blurred, edges, 50, 150);

//             // Find contours in the edge-detected image
//             std::vector<std::vector<cv::Point>> contours;
//             cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

//             // Filter contours based on the size and shape (we are looking for horizontal gaps)
//             for (const auto & contour : contours)
//             {
//                 // Get bounding box of the contour
//                 cv::Rect bounding_box = cv::boundingRect(contour);

//                 // Define conditions for considering this a gap (e.g., width > height and large enough)
//                 if (bounding_box.width > bounding_box.height && bounding_box.width > 100 && bounding_box.height < 50)
//                 {
//                     // Draw a rectangle around the detected gap
//                     cv::rectangle(img, bounding_box, cv::Scalar(0, 255, 0), 2);

//                     // Optionally print or log the location of the gap
//                    // RCLCPP_INFO(this->get_logger(), "Gap detected at (x: %d, y: %d), width: %d, height: %d",
//                                // bounding_box.x, bounding_box.y, bounding_box.width, bounding_box.height);
//                 }
//             }

//             // Display the processed image with detected gaps
//             cv::imshow("Detected Gaps", img);
//             cv::waitKey(1);  // Needed for OpenCV window updates
//         }
//         catch (cv_bridge::Exception& e)
//         {
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//         }
//     }


//     void move() {
//         while(true) {
//             std::cout << "input a command" << std::endl;
//             std::cin >> n_;

//             if (n_ == 'e') {
//                 std::cout << "n = e, stop now!" << std::endl;
//                 estopped = !estopped;
//                 move_pub->publish(stop);
//             }
//             if (!estopped) {
//                 if (n_ == 'w') {
//                     if (safe) {
//                         std::cout << "n = w, move forward" << std::endl;
//                         move_pub->publish(forward);
//                     }
//                     else {
//                         std::cout << "unsafe to move forwards" << std::endl;
//                         move_pub->publish(stop);
//                     }
//                 }
//                 else if (n_ == 'a') {
//                     std::cout << "n = a, move left" << std::endl;
//                     move_pub->publish(left);
//                 }
//                 else if (n_ == 'd') {
//                     std::cout << "n = d, move right" << std::endl;
//                     move_pub->publish(right);
//                 }
//             }
            
//         }

        
//     }


//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
//     sensor_msgs::msg::LaserScan laserScan_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_pub;
//     //rclcpp::TimerBase::SharedPtr timer_; // timer
//     std::thread user_input_thread_; // thread to capture user input
//     char n_;
//     geometry_msgs::msg::Twist stop;
//     geometry_msgs::msg::Twist forward;
//     geometry_msgs::msg::Twist backward;
//     geometry_msgs::msg::Twist left;
//     geometry_msgs::msg::Twist right;
//     bool safe;
//     bool stopped;
//     bool estopped;

// };
 
// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<LaserScan>();
//     //auto node = std::make_shared<CameraGapDetection>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
 
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraBinAndDoorwayDetection : public rclcpp::Node
{
public:
    CameraBinAndDoorwayDetection() : Node("camera_bin_and_doorway_detection")
    {
        // Subscriber to the camera image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&CameraBinAndDoorwayDetection::imageCallback, this, std::placeholders::_1));
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
            detectDoorways(img);

            // Display the processed image with detected bins and doorways
            cv::imshow("Detected Bins and Doorways", img);
            cv::waitKey(1);  // Needed for OpenCV window updates
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void detectDoorways(cv::Mat& img) {
        // Convert image to grayscale for easier processing
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian Blur to reduce noise
        cv::Mat blurred;
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

        // Apply Canny edge detection
        cv::Mat edges;
        cv::Canny(blurred, edges, 50, 150);

        // Find contours in the edge-detected image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Filter contours based on the size and shape (we are looking for horizontal gaps)
        for (const auto & contour : contours)
        {
            // Get bounding box of the contour
            cv::Rect bounding_box = cv::boundingRect(contour);

            // Define conditions for considering this a gap (e.g., width > height and large enough)
            if (bounding_box.width > bounding_box.height && bounding_box.width > 100 && bounding_box.height < 50)
            {
                // Draw a rectangle around the detected gap
                cv::rectangle(img, bounding_box, cv::Scalar(0, 255, 0), 2);

                // Optionally print or log the location of the gap
                // RCLCPP_INFO(this->get_logger(), "Gap detected at (x: %d, y: %d), width: %d, height: %d",
                            // bounding_box.x, bounding_box.y, bounding_box.width, bounding_box.height);
            }
        }

        // Display the processed image with detected gaps
       // cv::imshow("Detected Gaps", img);
        cv::waitKey(1);  // Needed for OpenCV window updates
    }

    // // Detect bins using color filtering and contour detection
    // void detectBins(cv::Mat& img)
    // {
    //     // Convert the image to HSV color space
    //     cv::Mat hsv_image;
    //     cv::cvtColor(img, hsv_image, cv::COLOR_BGR2HSV);

    //     // Define the range for the color of the bins (e.g., green bins)
    //     cv::Scalar lower_green(35, 100, 100);  // Lower HSV bounds for green
    //     cv::Scalar upper_green(85, 255, 255);  // Upper HSV bounds for green

    //     // Threshold the image to isolate green bins
    //     cv::Mat mask;
    //     cv::inRange(hsv_image, lower_green, upper_green, mask);

    //     // Optional: apply morphological operations to clean up noise
    //     cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    //     cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    //     // Find contours of the green objects (bins)
    //     std::vector<std::vector<cv::Point>> contours;
    //     cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    //     // Loop through the contours and find bins (based on contour area or shape)
    //     for (size_t i = 0; i < contours.size(); i++)
    //     {
    //         // Approximate the contour to reduce noise
    //         std::vector<cv::Point> approx;
    //         cv::approxPolyDP(contours[i], approx, 0.02 * cv::arcLength(contours[i], true), true);

    //         // Check if the contour has enough area to be considered a bin
    //         if (cv::contourArea(approx) > 500)  // Adjust this threshold for bin size
    //         {
    //             // Draw bounding box around detected bins
    //             cv::Rect bounding_box = cv::boundingRect(approx);
    //             cv::rectangle(img, bounding_box, cv::Scalar(0, 255, 0), 2);

    //             // Optionally, log the bin's location
    //             RCLCPP_INFO(this->get_logger(), "Bin detected at (x: %d, y: %d), width: %d, height: %d",
    //                         bounding_box.x, bounding_box.y, bounding_box.width, bounding_box.height);
    //         }
    //     }
    // }

    // Detect doorways using edge detection and line detection
    void detectBins(cv::Mat& img)
    {
        // Convert the image to grayscale for easier edge detection
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian Blur to reduce noise
        cv::Mat blurred;
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

        // Apply Canny edge detection
        cv::Mat edges;
        cv::Canny(blurred, edges, 50, 150);

        // Find contours of the edges (for detecting doorways)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Loop through the contours to detect doorways based on size and shape
        for (size_t i = 0; i < contours.size(); i++)
        {
            // Approximate the contour to reduce noise
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[i], approx, 0.02 * cv::arcLength(contours[i], true), true);

            // Check if the contour has a doorway-like shape (e.g., a tall rectangular shape)
            cv::Rect bounding_box = cv::boundingRect(approx);
            if (bounding_box.width < bounding_box.height && bounding_box.height > 100 && bounding_box.width > 50)
            {
                // Draw bounding box around detected doorways
                cv::rectangle(img, bounding_box, cv::Scalar(255, 0, 0), 2);

                // Optionally, log the doorway's location
                RCLCPP_INFO(this->get_logger(), "Doorway detected at (x: %d, y: %d), width: %d, height: %d",
                            bounding_box.x, bounding_box.y, bounding_box.width, bounding_box.height);
            }
        }
    }

    // Subscriber to the camera image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraBinAndDoorwayDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
