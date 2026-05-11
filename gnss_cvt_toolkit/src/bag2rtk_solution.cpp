#include <cstdio>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>

namespace
{

constexpr const char * kDefaultTopic = "/ublox_driver/receiver_pvt";

struct RTKState
{
  uint64_t gnss_ts_ns;
  Eigen::Vector3d t_ecef;
  Eigen::Vector3d v_enu;
  uint8_t fix_type;
  bool valid_fix;
  bool diff_soln;
  uint8_t carr_soln;
};
using RTKStatePtr = std::shared_ptr<RTKState>;

template<typename MessageT>
MessageT read_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serialization;
  MessageT message;
  serialization.deserialize_message(&serialized_message, &message);
  return message;
}

void extract_rtk_states(
  const std::string & bag_path,
  const std::string & topic_name,
  std::vector<RTKStatePtr> & all_states)
{
  all_states.clear();

  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = {topic_name};
  reader.set_filter(storage_filter);

  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    if (bag_message->topic_name != topic_name) {
      continue;
    }

    auto pvt_msg = read_message<gnss_comm::msg::GnssPVTSolnMsg>(bag_message);
    auto pvt = gnss_comm::msg2pvt(std::make_shared<gnss_comm::msg::GnssPVTSolnMsg>(std::move(pvt_msg)));

    auto rtk_state = std::make_shared<RTKState>();
    rtk_state->gnss_ts_ns = static_cast<uint64_t>(gnss_comm::time2sec(pvt->time) * 1e9);
    const Eigen::Vector3d pvt_lla(pvt->lat, pvt->lon, pvt->hgt);
    rtk_state->t_ecef = gnss_comm::geo2ecef(pvt_lla);
    rtk_state->v_enu.x() = pvt->vel_e;
    rtk_state->v_enu.y() = pvt->vel_n;
    rtk_state->v_enu.z() = -pvt->vel_d;
    rtk_state->fix_type = pvt->fix_type;
    rtk_state->valid_fix = pvt->valid_fix;
    rtk_state->diff_soln = pvt->diff_soln;
    rtk_state->carr_soln = pvt->carr_soln;
    all_states.push_back(std::move(rtk_state));
  }
}

bool save_states(const std::vector<RTKStatePtr> & states, const std::string & filepath)
{
  FILE * fp = fopen(filepath.c_str(), "w");
  if (fp == nullptr) {
    std::perror(("Failed to open output file " + filepath).c_str());
    return false;
  }

  for (const auto & state : states) {
    fprintf(fp, "%llu", static_cast<unsigned long long>(state->gnss_ts_ns));
    for (int j = 0; j < 3; ++j) {
      fprintf(fp, ", %.5f", state->t_ecef(j));
    }
    for (int j = 0; j < 3; ++j) {
      fprintf(fp, ", %.5f", state->v_enu(j));
    }
    fprintf(
      fp, ", %d, %d, %d, %d\n", static_cast<int>(state->fix_type),
      static_cast<int>(state->valid_fix), static_cast<int>(state->diff_soln),
      static_cast<int>(state->carr_soln));
  }

  fclose(fp);
  return true;
}

void print_usage(const char * program_name)
{
  std::cerr << "Usage: " << program_name << " <input_bag_dir> <output_csv_file> [topic_name]\n";
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
  const std::string output_rtk_path = argv[2];
  const std::string topic_name = argc == 4 ? argv[3] : kDefaultTopic;

  std::vector<RTKStatePtr> all_states;
  extract_rtk_states(bag_path, topic_name, all_states);

  if (!save_states(all_states, output_rtk_path)) {
    rclcpp::shutdown();
    return 1;
  }

  std::cout << "Converted " << all_states.size() << " RTK states to "
            << output_rtk_path << '\n';

  rclcpp::shutdown();
  return 0;
}
