cmake_minimum_required(VERSION 3.8)
project(rs1_group_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
add_definitions(-DQT_NO_KEYWORDS)  # Needed for Qt5
set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTORCC ON)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(userInput src/userInput.cpp)
ament_target_dependencies(userInput rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs cv_bridge nav2_msgs image_transport OpenCV)
add_executable(navigation src/navigation.cpp)
ament_target_dependencies(navigation rclcpp std_msgs sensor_msgs geometry_msgs)
add_executable(detect src/objectDetection.cpp)
ament_target_dependencies(detect rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs cv_bridge image_transport OpenCV)

add_executable(detect_copy src/detect_copy.cpp)
ament_target_dependencies(detect_copy rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs cv_bridge nav2_msgs image_transport OpenCV)

# add_executable(gui_application src/gui_application.cpp)

# # Link Qt and ROS 2 libraries
# target_link_libraries(gui_application Qt5::Widgets)
# ament_target_dependencies(gui_application rclcpp geometry_msgs)

install(TARGETS
  userInput
  detect
  detect_copy
  # gui_application
  DESTINATION lib/rs1_group_project)
  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch worlds maps
  DESTINATION share/rs1_group_project
)

ament_package()
