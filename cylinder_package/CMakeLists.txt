cmake_minimum_required(VERSION 3.8)
project(cylinder_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(cylinder_rotate src/cylinder_rotate.cpp)

# Link libraries
ament_target_dependencies(cylinder_rotate
    rclcpp 
    sensor_msgs 
    geometry_msgs 
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    nav2_msgs
    std_srvs
    rclcpp_action
)
add_executable(initial_pose src/initial_pose.cpp)
ament_target_dependencies(initial_pose rclcpp geometry_msgs)

# Install executable
install(TARGETS
  cylinder_rotate
  initial_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch world maps rviz
  DESTINATION share/cylinder_package
)

ament_package()
