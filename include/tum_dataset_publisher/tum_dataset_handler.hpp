#ifndef TUM_DATASET_HANDLER_HPP
#define TUM_DATASET_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <tuple>

class TUMDatasetHandler {
public:
    TUMDatasetHandler(rclcpp::Node::SharedPtr node, const std::string& dataset_path, const std::string& dataset_type, bool publish_groundtruth, bool publish_accelerometer);
    void publishData();

private:
    void loadDataset(const std::string& path);
    void publishImages(const std::string& rgb_path, const std::string& depth_path, double timestamp);

    rclcpp::Node::SharedPtr node_;
    image_transport::Publisher rgb_pub_;
    image_transport::Publisher depth_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr groundtruth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accelerometer_pub_;

    std::vector<double> timestamps_;
    std::vector<std::string> rgb_image_paths_;
    std::vector<std::string> depth_image_paths_;
    std::vector<std::tuple<double, double, double, double, double, double, double, double>> groundtruth_data_;
    std::vector<std::tuple<double, double, double, double>> accelerometer_data_;

    bool publish_groundtruth_;
    bool publish_accelerometer_;
};

#endif // TUM_DATASET_HANDLER_HPP