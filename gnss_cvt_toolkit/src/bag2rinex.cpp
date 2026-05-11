#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/rinex_helper.hpp>

namespace
{

constexpr const char * kDefaultTopic = "/ublox_driver/range_meas";

template<typename MessageT>
MessageT read_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serialization;
  MessageT message;
  serialization.deserialize_message(&serialized_message, &message);
  return message;
}

std::vector<std::vector<gnss_comm::ObsPtr>> parse_gnss_meas(
  const std::string & bag_path,
  const std::string & topic_name)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = {topic_name};
  reader.set_filter(storage_filter);

  std::vector<std::vector<gnss_comm::ObsPtr>> result;
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    if (bag_message->topic_name != topic_name) {
      continue;
    }

    auto obs_msg = read_message<gnss_comm::msg::GnssMeasMsg>(bag_message);
    result.push_back(gnss_comm::msg2meas(std::make_shared<gnss_comm::msg::GnssMeasMsg>(std::move(obs_msg))));
  }
  return result;
}

void print_usage(const char * program_name)
{
  std::cerr << "Usage: " << program_name << " <input_bag_dir> <output_rinex_file> [topic_name]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 3 || argc > 4) {
    print_usage(argv[0]);
    rclcpp::shutdown();
    return 1;
  }

  const std::string bag_path = argv[1];
  const std::string output_rinex_path = argv[2];
  const std::string topic_name = argc == 4 ? argv[3] : kDefaultTopic;

  const auto all_gnss_meas = parse_gnss_meas(bag_path, topic_name);
  gnss_comm::obs2rinex(output_rinex_path, all_gnss_meas);

  std::cout << "Converted " << all_gnss_meas.size() << " GNSS measurement epochs to "
            << output_rinex_path << '\n';

  rclcpp::shutdown();
  return 0;
}
