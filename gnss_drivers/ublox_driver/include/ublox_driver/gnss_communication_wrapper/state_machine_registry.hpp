/**
 * @brief Lightweight runtime state-machine registry for persistent driver conditions.
 */
#ifndef UBLOX_DRIVER_STATE_MACHINE_REGISTRY_HPP_
#define UBLOX_DRIVER_STATE_MACHINE_REGISTRY_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>

namespace ublox_driver {

inline constexpr char kStateMachineGlonassUnknownSvid255[] = "GLONASS_UNKNOWN_SVID_255";
inline constexpr char kStateMachineNtripConnected[] = "NTRIP_CONNECTED";
inline constexpr char kStateMachineRtcmStreamActive[] = "RTCM_STREAM_ACTIVE";

enum class StateMachineState {
  Deactive,
  Active,
};

struct StateMachineDefinition {
  std::string id;
  std::string display_name;
  std::chrono::milliseconds deactivate_timeout{0};
};

class StateMachineRegistry {
public:
  void registerDefinition(StateMachineDefinition definition);

  bool reportEvent(const std::string &definition_id,
                   const std::string &instance_key = "",
                   const std::string &detail = "");
  bool setActive(const std::string &definition_id,
                 const std::string &instance_key = "",
                 const std::string &detail = "");
  bool setDeactive(const std::string &definition_id,
                   const std::string &instance_key = "",
                   const std::string &detail = "");
  bool tick();

  std::string renderStatusList() const;

private:
  struct StateMachineInstance {
    StateMachineState state = StateMachineState::Deactive;
    uint64_t active_count = 0;
    std::chrono::steady_clock::time_point last_event_time {};
    std::chrono::steady_clock::time_point last_transition_time {};
    std::string last_detail;
  };

  std::string composeInstanceId(const std::string &definition_id,
                                const std::string &instance_key) const;
  StateMachineInstance &ensureInstanceLocked(const std::string &definition_id,
                                             const std::string &instance_key);
  bool setStateLocked(StateMachineInstance &instance,
                      StateMachineState new_state,
                      const std::string &detail,
                      const std::chrono::steady_clock::time_point &now);
  static const char *stateColor(StateMachineState state);
  static const char *stateLabel(StateMachineState state);

  mutable std::mutex mutex_;
  std::map<std::string, StateMachineDefinition> definitions_;
  std::map<std::string, StateMachineInstance> instances_;
};

} // namespace ublox_driver

#endif
