#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class GUIApp : public QMainWindow {
  Q_OBJECT

public:
  GUIApp(std::shared_ptr<rclcpp::Node> node, QWidget *parent = 0) : QMainWindow(parent), node_(node) {
    // Create a publisher for velocity commands
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create buttons for robot control
    QPushButton *forwardButton = new QPushButton("Move Forward", this);
    QPushButton *stopButton = new QPushButton("Stop", this);

    connect(forwardButton, SIGNAL(clicked()), this, SLOT(moveForward()));
    connect(stopButton, SIGNAL(clicked()), this, SLOT(stopRobot()));

    // Set button positions on the GUI window
    forwardButton->setGeometry(QRect(QPoint(50, 50), QSize(200, 50)));
    stopButton->setGeometry(QRect(QPoint(50, 150), QSize(200, 50)));
  }
  ~GUIApp(){}

private Q_SLOTS:
  void moveForward(){
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.5;  // Move forward
    cmd_vel_pub_->publish(msg);
  }
  void stopRobot(){
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;  // Stop
    cmd_vel_pub_->publish(msg);
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("turtlebot_qt_control");

  // Initialize the Qt Application
  QApplication app(argc, argv);
  GUIApp window(node);
  window.show();

  // Start the ROS 2 executor in a separate thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros_thread([&executor]() {
    executor.spin();
  });

  // Run the Qt event loop
  int result = app.exec();

  // Clean up
  rclcpp::shutdown();
  ros_thread.join();
  
  return result;
}
