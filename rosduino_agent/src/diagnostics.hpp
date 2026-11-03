// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_AGENT_DIAGNOSTICS_HPP_
#define ROSDUINO_AGENT_DIAGNOSTICS_HPP_

#include <memory>
#include <string>

#include "bridge/agent_session.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "transport/transport.h"

namespace rosduino {
namespace agent {

class DiagnosticsPublisher {
 public:
  DiagnosticsPublisher(rclcpp::Node::SharedPtr node, host::AgentSession& session,
                       host::Transport& transport, std::string port);

 private:
  void publish();

  rclcpp::Node::SharedPtr node_;
  host::AgentSession& session_;
  host::Transport& transport_;
  std::string port_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agent
}  // namespace rosduino

#endif  // ROSDUINO_AGENT_DIAGNOSTICS_HPP_
