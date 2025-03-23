#include "tum_dataset_publisher/tum_dataset_handler.hpp"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <sstream>

TUMDatasetHandler::TUMDatasetHandler(rclcpp::Node::SharedPtr node, const std::string& dataset_path, const std::string& dataset_type, bool publish_groundtruth, bool publish_accelerometer)
    : node_(node), publish_groundtruth_(publish_groundtruth), publish_accelerometer_(publish_accelerometer)
{
    image_transport::ImageTransport it(node_);
    rgb_pub_ = it.advertise("/camera/rgb/image_raw", 10);
    depth_pub_ = it.advertise("/camera/depth/image_raw", 10);
    groundtruth_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/groundtruth", 10);
    accelerometer_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/accelerometer", 10);
    loadDataset(dataset_path);
}

void TUMDatasetHandler::loadDataset(const std::string& path)
{
    std::ifstream rgb_file(path + "/rgb.txt");
    std::ifstream depth_file(path + "/depth.txt");
    std::ifstream groundtruth_file(path + "/groundtruth.txt");
    std::ifstream accelerometer_file(path + "/accelerometer.txt");

    if (!rgb_file.is_open() || !depth_file.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open rgb.txt or depth.txt at path: %s", path.c_str());
        return;
    }

    std::string rgb_line, depth_line;
    while (std::getline(rgb_file, rgb_line) && std::getline(depth_file, depth_line)) {
        // Skip comments and empty lines
        if (rgb_line.empty() || rgb_line[0] == '#' || depth_line.empty() || depth_line[0] == '#') {
            continue;
        }

        std::istringstream rgb_stream(rgb_line), depth_stream(depth_line);
        std::string rgb_timestamp, rgb_path, depth_timestamp, depth_path;

        if (!(rgb_stream >> rgb_timestamp >> rgb_path) || !(depth_stream >> depth_timestamp >> depth_path)) {
            RCLCPP_WARN(node_->get_logger(), "Malformed line in dataset: %s or %s", rgb_line.c_str(), depth_line.c_str());
            continue;
        }

        try {
            timestamps_.push_back(std::stod(rgb_timestamp));
            rgb_image_paths_.push_back(path + "/" + rgb_path);
            depth_image_paths_.push_back(path + "/" + depth_path);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to parse timestamp: %s", rgb_timestamp.c_str());
        }
    }

    if (publish_groundtruth_ && groundtruth_file.is_open()) {
        std::string groundtruth_line;
        while (std::getline(groundtruth_file, groundtruth_line)) {
            if (groundtruth_line.empty() || groundtruth_line[0] == '#') continue;
            std::istringstream gt_stream(groundtruth_line);
            double timestamp, tx, ty, tz, qx, qy, qz, qw;
            if (gt_stream >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
                groundtruth_data_.emplace_back(timestamp, tx, ty, tz, qx, qy, qz, qw);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Malformed groundtruth line: %s", groundtruth_line.c_str());
            }
        }
    }

    if (publish_accelerometer_ && accelerometer_file.is_open()) {
        std::string accelerometer_line;
        while (std::getline(accelerometer_file, accelerometer_line)) {
            if (accelerometer_line.empty() || accelerometer_line[0] == '#') continue;
            std::istringstream acc_stream(accelerometer_line);
            double timestamp, ax, ay, az;
            if (acc_stream >> timestamp >> ax >> ay >> az) {
                accelerometer_data_.emplace_back(timestamp, ax, ay, az);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Malformed accelerometer line: %s", accelerometer_line.c_str());
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Loaded %zu RGB-D frames.", timestamps_.size());
}


void TUMDatasetHandler::publishData()
{
    rclcpp::Rate rate(30); // Adjust frame rate
    for (size_t i = 0; i < rgb_image_paths_.size(); ++i) {
        publishImages(rgb_image_paths_[i], depth_image_paths_[i], timestamps_[i]);

        if (publish_groundtruth_ && i < groundtruth_data_.size()) {
            auto& [timestamp, tx, ty, tz, qx, qy, qz, qw] = groundtruth_data_[i];
            geometry_msgs::msg::PoseStamped gt_msg;
            gt_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
            gt_msg.pose.position.x = tx;
            gt_msg.pose.position.y = ty;
            gt_msg.pose.position.z = tz;
            gt_msg.pose.orientation.x = qx;
            gt_msg.pose.orientation.y = qy;
            gt_msg.pose.orientation.z = qz;
            gt_msg.pose.orientation.w = qw;
            groundtruth_pub_->publish(gt_msg);
        }

        if (publish_accelerometer_ && i < accelerometer_data_.size()) {
            auto& [timestamp, ax, ay, az] = accelerometer_data_[i];
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
            imu_msg.linear_acceleration.x = ax;
            imu_msg.linear_acceleration.y = ay;
            imu_msg.linear_acceleration.z = az;
            accelerometer_pub_->publish(imu_msg);
        }

        rate.sleep();
    }
}

void TUMDatasetHandler::publishImages(const std::string& rgb_path, const std::string& depth_path, double timestamp)
{
    cv::Mat rgb_image = cv::imread(rgb_path, cv::IMREAD_COLOR);
    cv::Mat depth_image = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
    if (rgb_image.empty() || depth_image.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Failed to load images: %s, %s", rgb_path.c_str(), depth_path.c_str());
        return;
    }

    auto rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_image).toImageMsg();
    auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image).toImageMsg();
    rgb_msg->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
    depth_msg->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));

    rgb_pub_.publish(rgb_msg);
    depth_pub_.publish(depth_msg);
}