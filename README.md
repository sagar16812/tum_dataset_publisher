# TUM Data Publisher ROS2 Package

The main objective of creating this package was to bridge the gap between the TUM datasets and ROS 2. While TUM provides rosbag files, they are not directly compatible with ROS 2, making it challenging to use these datasets for testing SLAM and other applications. Additionally, the datasets lack proper ROS 2 message structures, making it difficult to publish sensor data like RGB images, depth images, IMU readings, and ground truth in a way that integrates smoothly with ROS 2-based systems. This package addresses these issues by providing a simple and effective way to publish TUM dataset sequences as ROS 2 topics, enabling researchers and developers to test their SLAM algorithms and other robotics applications seamlessly. Currently, it supports the **RGB-D SLAM Dataset and Benchmark**, with future plans to add support for:
- Monocular Visual Odometry Dataset
- 4Seasons Dataset
- Visual Inertial Dataset

## Package Structure
```
.
├── CMakeLists.txt
├── config
│   ├── tum_dataset_publisher.rviz
│   └── tum_params.yaml
├── include
│   └── tum_dataset_publisher
│       └── tum_dataset_handler.hpp
├── launch
│   └── tum_publisher.launch.py
├── package.xml
├── README.md
└── src
    ├── tum_dataset_handler.cpp
    └── tum_publisher_node.cpp
```

## Installation
1. Clone this repository into your ROS2 workspace `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/sagar16812/tum_dataset_publisher.git
   ```
2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
3. Source the setup script:
   ```bash
   source install/setup.bash
   ```

## Usage
Launch the dataset publisher with the following command:
```bash
ros2 launch tum_dataset_publisher tum_publisher.launch.py \
    dataset_type:=rgbd \
    sequence_path:=/path/to/your/Dataset/sequence \
    frame_rate:=30 \
    groundtruth:=true \
    accelerometer:=true \
    visualize:=true
```

### Launch Options
- **dataset_type**: Type of dataset (currently supports only `rgbd`).
- **sequence_path**: Path to your dataset sequence.
- **frame_rate**: Frame rate for data publishing (default: 30).
- **groundtruth**: Publish ground truth poses (`true` or `false`).
- **accelerometer**: Publish IMU data (`true` or `false`).
- **visualize**: Enable visualization in `rviz2` (`true` or `false`).

## Visualizing with RViz2
If you want to visualize the published data with RViz2, ensure the `visualize` argument is set to `true`. The package includes a pre-configured RViz file `tum_dataset_publisher.rviz` located in the `config` directory.

To launch RViz with the saved configuration:
```bash
ros2 launch tum_dataset_publisher tum_publisher.launch.py visualize:=true
```

## Saving Configuration
If you modify the RViz settings, save the configuration as:
```bash
rviz2 -d <path_to_tum_dataset_publisher.rviz>
```
Then click **File > Save Config** and overwrite the `tum_dataset_publisher.rviz` file in the `config` folder.

## Troubleshooting
- Ensure you’ve sourced the workspace with `source install/setup.bash`.
- Verify dataset paths and frame rates.
- Check the `tum_dataset_publisher.rviz` file path if RViz fails to load.

## Contributing
Feel free to fork the repository and submit pull requests. Contributions are welcome!

## License
This project is licensed under the MIT License.

## Acknowledgments
- TUM Dataset: https://vision.in.tum.de/data/datasets