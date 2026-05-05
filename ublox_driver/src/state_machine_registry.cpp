#include "ublox_driver/state_machine_registry.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ublox_driver {

namespace {

constexpr char kAnsiRed[] = "\033[31m";
constexpr char kAnsiGreen[] = "\033[32m";
constexpr char kAnsiReset[] = "\033[0m";

} // namespace

void StateMachineRegistry::registerDefinition(StateMachineDefinition definition) {
  if (definition.id.empty()) {
    throw std::invalid_argument("State machine definition id cannot be empty.");
  }
  if (definition.display_name.empty()) {
    definition.display_name = definition.id;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  definitions_[definition.id] = definition;
  (void)ensureInstanceLocked(definition.id, "");
}

bool StateMachineRegistry::reportEvent(const std::string &definition_id,
                                       const std::string &instance_key,
                                       const std::string &detail) {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  StateMachineInstance &instance = ensureInstanceLocked(definition_id, instance_key);
  instance.active_count += 1U;
  instance.last_event_time = now;
  return setStateLocked(instance, StateMachineState::Active, detail, now);
}

bool StateMachineRegistry::setActive(const std::string &definition_id,
                                     const std::string &instance_key,
                                     const std::string &detail) {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  StateMachineInstance &instance = ensureInstanceLocked(definition_id, instance_key);
  instance.last_event_time = now;
  return setStateLocked(instance, StateMachineState::Active, detail, now);
}

bool StateMachineRegistry::setDeactive(const std::string &definition_id,
                                       const std::string &instance_key,
                                       const std::string &detail) {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  StateMachineInstance &instance = ensureInstanceLocked(definition_id, instance_key);
  return setStateLocked(instance, StateMachineState::Deactive, detail, now);
}

bool StateMachineRegistry::tick() {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);

  bool changed = false;
  for (const auto &[definition_id, definition] : definitions_) {
    (void)definition_id;
    if (definition.deactivate_timeout.count() <= 0) {
      continue;
    }

    for (auto &[instance_id, instance] : instances_) {
      if (instance_id.rfind(definition.id, 0) != 0) {
        continue;
      }
      if (instance.state != StateMachineState::Active) {
        continue;
      }
      if ((now - instance.last_event_time) < definition.deactivate_timeout) {
        continue;
      }

      changed = setStateLocked(instance, StateMachineState::Deactive,
                               "timeout", now) || changed;
    }
  }
  return changed;
}

std::string StateMachineRegistry::renderStatusList() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::ostringstream oss;
  oss << "State Machine List:";
  for (const auto &[definition_id, definition] : definitions_) {
    (void)definition_id;
    for (const auto &[instance_id, instance] : instances_) {
      if (instance_id.rfind(definition.id, 0) != 0) {
        continue;
      }

      oss << "\n  "
          << stateColor(instance.state)
          << '[' << std::left << std::setw(8) << stateLabel(instance.state) << ']'
          << ' ' << definition.display_name;

      if (instance_id.size() > definition.id.size() + 1U) {
        oss << " key=" << instance_id.substr(definition.id.size() + 1U);
      }

      oss << " count=" << instance.active_count;
      if (!instance.last_detail.empty()) {
        oss << " detail=" << instance.last_detail;
      }
      oss << kAnsiReset;
    }
  }

  return oss.str();
}

std::string StateMachineRegistry::composeInstanceId(
    const std::string &definition_id, const std::string &instance_key) const {
  if (instance_key.empty()) {
    return definition_id;
  }
  return definition_id + ":" + instance_key;
}

StateMachineRegistry::StateMachineInstance &
StateMachineRegistry::ensureInstanceLocked(const std::string &definition_id,
                                           const std::string &instance_key) {
  if (definitions_.find(definition_id) == definitions_.end()) {
    throw std::invalid_argument("Unknown state machine definition id: " + definition_id);
  }

  return instances_[composeInstanceId(definition_id, instance_key)];
}

bool StateMachineRegistry::setStateLocked(
    StateMachineInstance &instance,
    StateMachineState new_state,
    const std::string &detail,
    const std::chrono::steady_clock::time_point &now) {
  const bool changed = instance.state != new_state;
  instance.state = new_state;
  instance.last_detail = detail;
  if (changed) {
    instance.last_transition_time = now;
  }
  return changed;
}

const char *StateMachineRegistry::stateColor(StateMachineState state) {
  return state == StateMachineState::Active ? kAnsiRed : kAnsiGreen;
}

const char *StateMachineRegistry::stateLabel(StateMachineState state) {
  return state == StateMachineState::Active ? "ACTIVE" : "DEACTIVE";
}

} // namespace ublox_driver
