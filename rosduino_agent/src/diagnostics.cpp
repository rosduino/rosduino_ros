// SPDX-License-Identifier: Apache-2.0
#include "diagnostics.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace rosduino {
namespace agent {
namespace {

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using rosduino::host::AgentSnapshot;

const char* profileName(uint8_t profile) {
  switch (profile) {
    case 1U:
      return "lite";
    case 2U:
      return "small";
    case 3U:
      return "medium";
    case 4U:
      return "large";
    default:
      return "default";
  }
}

void addKv(DiagnosticStatus* status, const char* key, const std::string& value) {
  KeyValue kv;
  kv.key = key;
  kv.value = value;
  status->values.push_back(std::move(kv));
}

void addKv(DiagnosticStatus* status, const char* key, uint32_t value) {
  addKv(status, key, std::to_string(value));
}

void addKv(DiagnosticStatus* status, const char* key, uint8_t value) {
  addKv(status, key, static_cast<uint32_t>(value));
}

}  // namespace

DiagnosticsPublisher::DiagnosticsPublisher(rclcpp::Node::SharedPtr node, host::AgentSession& session,
                                           host::Transport& transport, std::string port)
    : node_(std::move(node)),
      session_(session),
      transport_(transport),
      port_(std::move(port)) {
  pub_ = node_->create_publisher<DiagnosticArray>("/diagnostics", 10);
  timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() { publish(); });
}

void DiagnosticsPublisher::publish() {
  const AgentSnapshot snap = session_.snapshot();
  const bool port_open = transport_.isOpen();

  DiagnosticStatus status;
  status.name = "rosduino: " + (snap.node_name.empty() ? std::string("unknown") : snap.node_name);
  status.hardware_id = port_;

  if (!port_open && snap.rx_frames == 0U && snap.node_name.empty()) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "port down";
  } else if (!port_open || !snap.active) {
    status.level = DiagnosticStatus::WARN;
    status.message = "no mcu link";
  } else if (snap.entity_errors > 0U) {
    status.level = DiagnosticStatus::WARN;
    status.message = "entity errors";
  } else {
    status.level = DiagnosticStatus::OK;
    status.message = "ok";
  }

  addKv(&status, "port", port_);
  addKv(&status, "session_state", snap.active ? "active" : "waiting_hello");
  addKv(&status, "proto_ver", snap.proto_ver);
  addKv(&status, "mcu_profile", profileName(snap.mcu_profile));
  addKv(&status, "entities", snap.entity_count);
  addKv(&status, "entity_errors", snap.entity_errors);
  addKv(&status, "publishers", snap.publishers);
  addKv(&status, "subscribers", snap.subscribers);
  addKv(&status, "service_servers", snap.service_servers);
  addKv(&status, "service_clients", snap.service_clients);
  addKv(&status, "rx_frames", snap.rx_frames);
  addKv(&status, "tx_frames", snap.tx_frames);
  addKv(&status, "crc_errors", snap.crc_errors);
  addKv(&status, "cobs_errors", snap.cobs_errors);
  addKv(&status, "losses", snap.losses);
  addKv(&status, "no_resources", snap.no_resources);
  addKv(&status, "too_large", snap.too_large);
  if (snap.last_pong_age_ms == 0xFFFFFFFFU) {
    addKv(&status, "last_pong_age_ms", "none");
  } else {
    addKv(&status, "last_pong_age_ms", snap.last_pong_age_ms);
  }

  DiagnosticArray array;
  array.header.stamp = node_->now();
  array.status.push_back(std::move(status));
  pub_->publish(array);
}

}  // namespace agent
}  // namespace rosduino
