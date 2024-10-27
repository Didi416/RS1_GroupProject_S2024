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

    void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    }
    
    //OUTLINE METHOD
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

    // Use Hough Line Transform to detect potential doorway edges
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 100, 50, 10);

    // Store vertical lines that could be potential doorway edges
    std::vector<cv::Vec4i> verticalLines;

    // Filter and store vertical lines based on x-coordinates
    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        
        // Consider line to be vertical if it is mostly straight up-down
        if (abs(x1 - x2) < 10 && abs(y1 - y2) > 50) {  // Adjust to fit doorway height
            verticalLines.push_back(line);
        }
    }

    // Pair vertical lines to outline a doorway
    for (size_t i = 0; i < verticalLines.size(); ++i) {
        for (size_t j = i + 1; j < verticalLines.size(); ++j) {
            int x1_1 = verticalLines[i][0];
            int x1_2 = verticalLines[j][0];

            // Check if the two lines are close enough horizontally to be the sides of a doorway
            if (abs(x1_1 - x1_2) > 350 && abs(x1_1 - x1_2) < 600) {  // Adjust the distance range as needed
                // Draw rectangles between the vertical lines
                cv::line(img, cv::Point(verticalLines[i][0], verticalLines[i][1]), 
                         cv::Point(verticalLines[i][2], verticalLines[i][3]), cv::Scalar(0, 255, 0), 2);
                cv::line(img, cv::Point(verticalLines[j][0], verticalLines[j][1]), 
                         cv::Point(verticalLines[j][2], verticalLines[j][3]), cv::Scalar(0, 255, 0), 2);

                // Draw a top line connecting the two vertical lines
                cv::line(img, cv::Point(verticalLines[i][0], verticalLines[i][1]), 
                         cv::Point(verticalLines[j][0], verticalLines[j][1]), cv::Scalar(0, 255, 0), 2);
            }
        }
    }

    // Display the processed image with detected doorway edges
    cv::imshow("Detected Doorway Edges", img);
    cv::waitKey(1);  // Needed for OpenCV window updates
}


//GAP METHOD
//    void detectDoorways(cv::Mat& img) {
//     // Convert the image to grayscale for easier processing
//     cv::Mat gray;
//     cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

//     // Apply Gaussian Blur to reduce noise
//     cv::Mat blurred;
//     cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

//     // Apply Canny edge detection
//     cv::Mat edges;
//     cv::Canny(blurred, edges, 50, 150);

//     // Define the region of interest (ROI) - lower part of the image
//     int roiHeight = img.rows / 2;  // Adjust based on how much floor is visible
//     cv::Rect roi(0, img.rows - roiHeight, img.cols, roiHeight);
//     cv::Mat roiEdges = edges(roi);

//     // Find contours in the ROI
//     std::vector<std::vector<cv::Point>> contours;
//     cv::findContours(roiEdges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

//     // Filter contours based on size and shape (detect horizontal gaps)
//     for (const auto& contour : contours)
//     {
//         // Get bounding box of the contour
//         cv::Rect bounding_box = cv::boundingRect(contour);

//         // Check if the bounding box meets conditions for a horizontal gap
//         if ( bounding_box.width > 350 && bounding_box.width < 600 || bounding_box.width > 650 && bounding_box.width < 1000)
//         {
//             // Draw a rectangle around the detected gap (shift to original image coordinates)
//             cv::rectangle(img, cv::Point(bounding_box.x, bounding_box.y + img.rows - roiHeight),
//                           cv::Point(bounding_box.x + bounding_box.width, bounding_box.y + bounding_box.height + img.rows - roiHeight),
//                           cv::Scalar(0, 255, 0), 2);

//             // Optionally print or log the location of the gap
//             RCLCPP_INFO(this->get_logger(), "Gap detected at (x: %d, y: %d), width: %d, height: %d",
//                         bounding_box.x, bounding_box.y + img.rows - roiHeight, bounding_box.width, bounding_box.height);
//         }
//     }

//     // Display the processed image with detected gaps
//     //cv::imshow("Detected Floor Gaps", img);
//     cv::waitKey(1);  // Needed for OpenCV window updates
// }

    
    // Detect bins using edge detection and line detection
    void detectBins(cv::Mat& img)
    {
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

        // Draw outlines around detected green areas (bins)
        for (size_t i = 0; i < contours.size(); i++)
        {
            // Draw the contour outline on the original image
            cv::drawContours(img, contours, (int)i, cv::Scalar(255, 0, 0), 2); // Red color with thickness 2
            //RCLCPP_INFO(this->get_logger(), "Bin contour detected, index: %zu", i);
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