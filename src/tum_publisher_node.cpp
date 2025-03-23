#include "tum_dataset_publisher/tum_dataset_handler.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tum_publisher_node");

    if (argc < 3) {
        RCLCPP_ERROR(node->get_logger(), "Usage: tum_publisher_node <dataset_type> <sequence_path> [groundtruth:=true] [accelerometer:=true]");
        rclcpp::shutdown();
        return 1;
    }

    std::string dataset_type = argv[1];
    std::string sequence_path = argv[2];
    bool publish_groundtruth = false;
    bool publish_accelerometer = false;

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "groundtruth:=true") {
            publish_groundtruth = true;
        } else if (arg == "accelerometer:=true") {
            publish_accelerometer = true;
        }
    }

    auto handler = std::make_shared<TUMDatasetHandler>(node, sequence_path, dataset_type, publish_groundtruth, publish_accelerometer);
    handler->publishData();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
