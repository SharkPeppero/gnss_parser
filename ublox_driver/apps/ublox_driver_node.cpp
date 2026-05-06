/**
 * @brief ublox_driver ROS 2 node entrypoint.
 */
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/utilities.hpp>

#include "ublox_driver/common/logging.hpp"
#include "ublox_driver/gnss_communication_wrapper/ublox_driver.hpp"

int main(int argc, char **argv) {
  const std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

  std::string config_filepath;
  std::string ublox_config_filepath;

  for (size_t i = 1; i < non_ros_args.size(); ++i) {
    const std::string &arg = non_ros_args[i];
    if (arg == "--config-path") {
      if (i + 1 >= non_ros_args.size()) {
        LOG(ERROR) << "Missing value after --config-path.";
        return 1;
      }
      config_filepath = non_ros_args[++i];
      continue;
    }
    if (arg == "--ublox-config-path") {
      if (i + 1 >= non_ros_args.size()) {
        LOG(ERROR) << "Missing value after --ublox-config-path.";
        return 1;
      }
      ublox_config_filepath = non_ros_args[++i];
      continue;
    }

    if (config_filepath.empty()) {
      config_filepath = arg;
      continue;
    }

    if (ublox_config_filepath.empty()) {
      ublox_config_filepath = arg;
      continue;
    }

    LOG(ERROR) << "Unrecognized extra argv: " << arg;
    return 1;
  }

  if (config_filepath.empty()) {
    LOG(ERROR) << "Missing driver config path argv. Please pass it from launch.";
    return 1;
  }

  LOG(INFO) << "config_filepath: " << config_filepath;
  LOG(INFO) << "ublox_config_filepath: " << ublox_config_filepath;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ublox_driver");
  auto ublox_driver_node = std::make_shared<ublox_driver::GNSSDriverManager>(node,            //
                                                                             config_filepath, //
                                                                             ublox_config_filepath);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
