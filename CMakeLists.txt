cmake_minimum_required(VERSION 3.8)
project(tum_dataset_publisher)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(tum_publisher_node src/tum_publisher_node.cpp src/tum_dataset_handler.cpp)
ament_target_dependencies(tum_publisher_node rclcpp sensor_msgs cv_bridge image_transport geometry_msgs OpenCV)

install(TARGETS tum_publisher_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()