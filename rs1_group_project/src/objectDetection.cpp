#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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

        depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", 10, std::bind(&CameraBinAndDoorwayDetection::depthCallback, this, std::placeholders::_1));
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

    void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

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
    void detectBins(cv::Mat& img)
    {
        // Convert the image to HSV color space
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);


       // Define the color range for the green bin (try a wider range)
        cv::Scalar lower_green(35, 50, 50);   // Adjusted lower bounds for Hue, Saturation, and Value
        cv::Scalar upper_green(85, 255, 255); // Adjusted upper bounds for Hue, Saturation, and Value


        // Create a mask for green colors
        cv::Mat mask;
        cv::inRange(hsv, lower_green, upper_green, mask);

        cv::imshow("Green Mask", mask);
        cv::waitKey(1);

        // Find contours in the green mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw outlines around detected green areas (bins)
        for (size_t i = 0; i < contours.size(); i++)
        {
            // Draw the contour outline on the original image
            cv::drawContours(img, contours, (int)i, cv::Scalar(255, 0, 0), 2); // Red color with thickness 2
            RCLCPP_INFO(this->get_logger(), "Bin contour detected, index: %zu", i);
        }
    
    }

    

    // Subscriber to the camera image topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraBinAndDoorwayDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}