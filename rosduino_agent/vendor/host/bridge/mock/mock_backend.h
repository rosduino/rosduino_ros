// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_MOCK_BACKEND_H_
#define ROSDUINO_HOST_MOCK_BACKEND_H_

#include <map>
#include <set>
#include <string>
#include <vector>

#include "bridge/ros_backend.h"

namespace rosduino {
namespace host {

struct MockCreateNode {
  std::string requested_name;
  std::string ns;
  std::string actual_name;
};

struct MockCreatePublisher {
  uint8_t id = 0U;
  TopicSpec spec;
};

struct MockCreateSubscriber {
  uint8_t id = 0U;
  TopicSpec spec;
};

struct MockCreateServiceServer {
  uint8_t id = 0U;
  ServiceSpec spec;
};

struct MockCreateServiceClient {
  uint8_t id = 0U;
  ServiceSpec spec;
};

struct MockServiceCall {
  uint8_t id = 0U;
  uint8_t req_id = 0U;
  std::vector<uint8_t> cdr;
};

struct MockPublish {
  uint8_t id = 0U;
  std::vector<uint8_t> cdr;
};

struct MockServiceResponse {
  uint8_t id = 0U;
  uint8_t req_id = 0U;
  std::vector<uint8_t> cdr;
};

struct MockLog {
  core::LogLevel level = core::LogLevel::Info;
  std::string text;
};

/**
 * @brief Records RosBackend calls for host tests (no ROS).
 */
class MockBackend final : public RosBackend {
 public:
  void registerType(const std::string& type, uint64_t hash) { type_hashes_[type] = hash; }

  std::string createNode(const std::string& name, const std::string& ns) override {
    std::string actual = name;
    int suffix = 2;
    while (live_nodes_.count(actual) != 0U) {
      actual = name + "_" + std::to_string(suffix++);
    }
    if (node_alive_ && !node_name_.empty()) {
      live_nodes_.erase(node_name_);
    }
    live_nodes_.insert(actual);
    node_alive_ = true;
    node_name_ = actual;
    node_ns_ = ns;
    node_calls_.push_back(MockCreateNode{name, ns, actual});
    return actual;
  }

  void destroyNode() override {
    if (node_alive_) {
      live_nodes_.erase(node_name_);
      node_alive_ = false;
      node_name_.clear();
      node_ns_.clear();
      ++destroy_node_count_;
    }
  }

  std::optional<uint64_t> lookupTypeHash(const std::string& type) const override {
    const auto it = type_hashes_.find(type);
    if (it == type_hashes_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  core::StatusCode createPublisher(uint8_t id, const TopicSpec& spec) override {
    pub_calls_.push_back(MockCreatePublisher{id, spec});
    publishers_[id] = spec;
    return core::StatusCode::Ok;
  }

  core::StatusCode createSubscriber(uint8_t id, const TopicSpec& spec) override {
    sub_calls_.push_back(MockCreateSubscriber{id, spec});
    subscribers_[id] = spec;
    return core::StatusCode::Ok;
  }

  void setServiceServerStatus(core::StatusCode code) { service_server_status_ = code; }

  core::StatusCode createServiceServer(uint8_t id, const ServiceSpec& spec) override {
    srv_calls_.push_back(MockCreateServiceServer{id, spec});
    if (service_server_status_ != core::StatusCode::Ok) {
      return service_server_status_;
    }
    servers_[id] = spec;
    return core::StatusCode::Ok;
  }

  core::StatusCode createServiceClient(uint8_t id, const ServiceSpec& spec) override {
    cli_calls_.push_back(MockCreateServiceClient{id, spec});
    clients_[id] = spec;
    return core::StatusCode::Ok;
  }

  void inject(uint8_t id, const uint8_t* cdr, size_t len) { deliverIncoming(id, cdr, len); }

  void injectServiceRequest(uint8_t id, uint8_t req_id, const uint8_t* cdr, size_t len) {
    deliverServiceToMcu(id, req_id, cdr, len);
  }

  void injectServiceFromRos(uint8_t id, uint8_t req_id, const uint8_t* cdr, size_t len) {
    deliverServiceFromRos(id, req_id, cdr, len);
  }

  core::StatusCode callService(uint8_t id, uint8_t req_id, const uint8_t* cdr,
                               size_t len) override {
    MockServiceCall call;
    call.id = id;
    call.req_id = req_id;
    if (cdr != nullptr && len > 0U) {
      call.cdr.assign(cdr, cdr + len);
    }
    service_calls_.push_back(std::move(call));
    return core::StatusCode::Ok;
  }

  core::StatusCode publish(uint8_t id, const uint8_t* cdr, size_t len) override {
    MockPublish call;
    call.id = id;
    if (cdr != nullptr && len > 0U) {
      call.cdr.assign(cdr, cdr + len);
    }
    publishes_.push_back(std::move(call));
    return core::StatusCode::Ok;
  }

  void completeServiceResponse(uint8_t id, uint8_t req_id, const uint8_t* cdr,
                               size_t len) override {
    MockServiceResponse call;
    call.id = id;
    call.req_id = req_id;
    if (cdr != nullptr && len > 0U) {
      call.cdr.assign(cdr, cdr + len);
    }
    service_responses_.push_back(std::move(call));
  }

  void destroyEntity(uint8_t id) override {
    publishers_.erase(id);
    subscribers_.erase(id);
    servers_.erase(id);
    clients_.erase(id);
    ++destroy_entity_count_;
  }

  void destroyAllEntities() override {
    publishers_.clear();
    subscribers_.clear();
    servers_.clear();
    clients_.clear();
    ++destroy_all_count_;
  }

  void log(core::LogLevel level, const std::string& text) override {
    logs_.push_back(MockLog{level, text});
  }

  bool nodeAlive() const { return node_alive_; }
  const std::string& nodeName() const { return node_name_; }
  const std::vector<MockCreateNode>& nodeCalls() const { return node_calls_; }
  const std::vector<MockCreatePublisher>& pubCalls() const { return pub_calls_; }
  const std::vector<MockCreateSubscriber>& subCalls() const { return sub_calls_; }
  const std::vector<MockCreateServiceServer>& srvCalls() const { return srv_calls_; }
  const std::vector<MockCreateServiceClient>& cliCalls() const { return cli_calls_; }
  const std::vector<MockPublish>& publishes() const { return publishes_; }
  const std::vector<MockServiceCall>& serviceCalls() const { return service_calls_; }
  const std::vector<MockServiceResponse>& serviceResponses() const { return service_responses_; }
  const std::vector<MockLog>& logs() const { return logs_; }
  int destroyEntityCount() const { return destroy_entity_count_; }
  int destroyAllCount() const { return destroy_all_count_; }
  int destroyNodeCount() const { return destroy_node_count_; }
  bool hasPublisher(uint8_t id) const { return publishers_.count(id) != 0U; }
  bool hasSubscriber(uint8_t id) const { return subscribers_.count(id) != 0U; }
  bool hasServiceServer(uint8_t id) const { return servers_.count(id) != 0U; }
  bool hasServiceClient(uint8_t id) const { return clients_.count(id) != 0U; }

 private:
  std::map<std::string, uint64_t> type_hashes_;
  std::set<std::string> live_nodes_;
  bool node_alive_ = false;
  std::string node_name_;
  std::string node_ns_;
  std::map<uint8_t, TopicSpec> publishers_;
  std::map<uint8_t, TopicSpec> subscribers_;
  std::map<uint8_t, ServiceSpec> servers_;
  std::map<uint8_t, ServiceSpec> clients_;
  std::vector<MockCreateNode> node_calls_;
  std::vector<MockCreatePublisher> pub_calls_;
  std::vector<MockCreateSubscriber> sub_calls_;
  std::vector<MockCreateServiceServer> srv_calls_;
  std::vector<MockCreateServiceClient> cli_calls_;
  std::vector<MockPublish> publishes_;
  std::vector<MockServiceCall> service_calls_;
  std::vector<MockServiceResponse> service_responses_;
  std::vector<MockLog> logs_;
  int destroy_entity_count_ = 0;
  int destroy_all_count_ = 0;
  int destroy_node_count_ = 0;
  core::StatusCode service_server_status_ = core::StatusCode::Ok;
};

}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_MOCK_BACKEND_H_
