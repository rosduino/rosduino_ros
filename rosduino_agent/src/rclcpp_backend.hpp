// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_AGENT_RCLCPP_BACKEND_HPP_
#define ROSDUINO_AGENT_RCLCPP_BACKEND_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "bridge/ros_backend.h"
#include "rclcpp/generic_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "type_registry.hpp"

#if defined(__has_include)
#if __has_include("rclcpp/generic_service.hpp")
#include "rclcpp/generic_service.hpp"
#define ROSDUINO_HAS_GENERIC_SERVICE 1
#endif
#endif

namespace rosduino {
namespace agent {

/**
 * @brief `RosBackend` over rclcpp generic publishers and subscriptions.
 */
class RclcppBackend final : public host::RosBackend {
 public:
  RclcppBackend(rclcpp::Node::SharedPtr agent_node, rclcpp::Executor& executor,
                std::string namespace_override, uint32_t service_timeout_ms = 5000U);

  std::string createNode(const std::string& name, const std::string& ns) override;
  void destroyNode() override;
  std::optional<uint64_t> lookupTypeHash(const std::string& type) const override;
  core::StatusCode createPublisher(uint8_t id, const host::TopicSpec& spec) override;
  core::StatusCode createSubscriber(uint8_t id, const host::TopicSpec& spec) override;
  core::StatusCode createServiceServer(uint8_t id, const host::ServiceSpec& spec) override;
  core::StatusCode createServiceClient(uint8_t id, const host::ServiceSpec& spec) override;
  core::StatusCode publish(uint8_t id, const uint8_t* cdr, size_t len) override;
  core::StatusCode callService(uint8_t id, uint8_t req_id, const uint8_t* cdr,
                               size_t len) override;
  void completeServiceResponse(uint8_t id, uint8_t req_id, const uint8_t* cdr,
                               size_t len) override;
  void pollServiceClients() override;
  void destroyEntity(uint8_t id) override;
  void destroyAllEntities() override;
  void log(core::LogLevel level, const std::string& text) override;

 private:
  std::string resolveNamespace(const std::string& hello_ns) const;
  std::string uniqueNodeName(const std::string& name, const std::string& ns) const;
  bool nodeNameTaken(const std::string& name, const std::string& ns) const;
  static std::string fullyQualified(const std::string& name, const std::string& ns);
  rclcpp::Logger logger() const;

  struct InflightRequest {
    uint8_t req_id = 0U;
    std::string type;
    std::shared_ptr<rmw_request_id_t> header;
    std::shared_ptr<void> response;
#if defined(ROSDUINO_HAS_GENERIC_SERVICE)
    rclcpp::GenericService::SharedPtr service;
#endif
  };

  struct PendingClientCall {
    uint8_t id = 0U;
    uint8_t req_id = 0U;
    std::string type;
    rclcpp::GenericClient::SharedPtr client;
    std::shared_ptr<void> request;
    std::chrono::steady_clock::time_point deadline;
    std::function<int()> poll;
    std::vector<uint8_t> cdr;
  };

  void failInflightLocked();
  void failInflight(uint8_t id);
  void cancelClientCallsLocked(uint8_t id, bool all);

  rclcpp::Node::SharedPtr agent_node_;
  rclcpp::Executor& executor_;
  std::string namespace_override_;
  uint32_t service_timeout_ms_;
  TypeRegistry types_;
  rclcpp::Node::SharedPtr mcu_node_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  std::mutex mu_;
  std::map<uint8_t, rclcpp::GenericPublisher::SharedPtr> publishers_;
  struct ClientSlot {
    rclcpp::GenericClient::SharedPtr client;
    std::string type;
  };

  std::map<uint8_t, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::map<uint8_t, ClientSlot> clients_;
#if defined(ROSDUINO_HAS_GENERIC_SERVICE)
  std::map<uint8_t, rclcpp::GenericService::SharedPtr> services_;
#endif
  std::mutex inflight_mu_;
  std::map<uint8_t, std::shared_ptr<InflightRequest>> inflight_;
  std::map<uint8_t, uint8_t> next_req_id_;
  std::mutex client_call_mu_;
  std::vector<std::shared_ptr<PendingClientCall>> pending_client_calls_;
};

}  // namespace agent
}  // namespace rosduino

#endif  // ROSDUINO_AGENT_RCLCPP_BACKEND_HPP_
