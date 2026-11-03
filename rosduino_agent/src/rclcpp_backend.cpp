// SPDX-License-Identifier: Apache-2.0
#include "rclcpp_backend.hpp"

#include <cstring>
#include <exception>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "bridge/qos.h"
#include "rclcpp/serialized_message.hpp"
#include "rosduino/core/commands.h"

namespace rosduino {
namespace agent {
namespace {

constexpr uint8_t kCdrEncap[4] = {0x00, 0x01, 0x00, 0x00};

rclcpp::QoS makeQos(const host::TopicSpec& spec) {
  const host::DecodedTopicQos decoded = host::decodeTopicQos(spec.qos, spec.depth);
  rclcpp::QoS qos{rclcpp::KeepLast(decoded.history_depth)};
  if (decoded.reliable) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  if (decoded.transient_local) {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }
  return qos;
}

}  // namespace

RclcppBackend::RclcppBackend(rclcpp::Node::SharedPtr agent_node, rclcpp::Executor& executor,
                             std::string namespace_override, uint32_t service_timeout_ms)
    : agent_node_(std::move(agent_node)),
      executor_(executor),
      namespace_override_(std::move(namespace_override)),
      service_timeout_ms_(service_timeout_ms == 0U ? 5000U : service_timeout_ms) {}

std::string RclcppBackend::fullyQualified(const std::string& name, const std::string& ns) {
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  if (ns.back() == '/') {
    return ns + name;
  }
  return ns + "/" + name;
}

std::string RclcppBackend::resolveNamespace(const std::string& hello_ns) const {
  if (!namespace_override_.empty()) {
    return namespace_override_;
  }
  return hello_ns;
}

bool RclcppBackend::nodeNameTaken(const std::string& name, const std::string& ns) const {
  const std::string fq = fullyQualified(name, ns);
  const std::vector<std::string> names = agent_node_->get_node_names();
  for (const std::string& n : names) {
    if (n == fq) {
      return true;
    }
  }
  return false;
}

std::string RclcppBackend::uniqueNodeName(const std::string& name, const std::string& ns) const {
  std::string actual = name;
  int suffix = 2;
  while (nodeNameTaken(actual, ns)) {
    actual = name + "_" + std::to_string(suffix++);
  }
  return actual;
}

rclcpp::Logger RclcppBackend::logger() const {
  if (mcu_node_) {
    return mcu_node_->get_logger();
  }
  return agent_node_->get_logger();
}

std::string RclcppBackend::createNode(const std::string& name, const std::string& ns) {
  destroyNode();
  const std::string resolved_ns = resolveNamespace(ns);
  const std::string actual = uniqueNodeName(name, resolved_ns);
  try {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    mcu_node_ = std::make_shared<rclcpp::Node>(actual, resolved_ns, options);
    service_cb_group_ =
        mcu_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    executor_.add_node(mcu_node_);
    return actual;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(agent_node_->get_logger(), "createNode(%s) failed: %s", name.c_str(), e.what());
    mcu_node_.reset();
    return std::string();
  }
}

void RclcppBackend::destroyNode() {
  destroyAllEntities();
  service_cb_group_.reset();
  if (mcu_node_) {
    executor_.remove_node(mcu_node_);
    mcu_node_.reset();
  }
}

std::optional<uint64_t> RclcppBackend::lookupTypeHash(const std::string& type) const {
  if (type.find("/srv/") != std::string::npos) {
    return types_.serviceHash(type);
  }
  return types_.messageHash(type);
}

core::StatusCode RclcppBackend::createPublisher(uint8_t id, const host::TopicSpec& spec) {
  if (!mcu_node_) {
    return core::StatusCode::NotInSession;
  }
  try {
    auto pub = mcu_node_->create_generic_publisher(spec.name, spec.type, makeQos(spec));
    if (!pub) {
      return core::StatusCode::RosError;
    }
    std::lock_guard<std::mutex> lock(mu_);
    publishers_[id] = std::move(pub);
    return core::StatusCode::Ok;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "create_generic_publisher(%s, %s) failed: %s", spec.name.c_str(),
                 spec.type.c_str(), e.what());
    return core::StatusCode::UnknownType;
  }
}

core::StatusCode RclcppBackend::createSubscriber(uint8_t id, const host::TopicSpec& spec) {
  if (!mcu_node_) {
    return core::StatusCode::NotInSession;
  }
  try {
    auto cb = [this, id](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      if (!msg) {
        return;
      }
      const auto& rcl_msg = msg->get_rcl_serialized_message();
      if (rcl_msg.buffer == nullptr || rcl_msg.buffer_length < sizeof(kCdrEncap)) {
        return;
      }
      if ((rcl_msg.buffer[1] & 0x01U) != 0x01U) {
        return;
      }
      deliverIncoming(id, rcl_msg.buffer + sizeof(kCdrEncap),
                      rcl_msg.buffer_length - sizeof(kCdrEncap));
    };
    auto sub = mcu_node_->create_generic_subscription(spec.name, spec.type, makeQos(spec), cb);
    if (!sub) {
      return core::StatusCode::RosError;
    }
    std::lock_guard<std::mutex> lock(mu_);
    subscriptions_[id] = std::move(sub);
    return core::StatusCode::Ok;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "create_generic_subscription(%s, %s) failed: %s", spec.name.c_str(),
                 spec.type.c_str(), e.what());
    return core::StatusCode::UnknownType;
  }
}

core::StatusCode RclcppBackend::createServiceClient(uint8_t id, const host::ServiceSpec& spec) {
  if (!mcu_node_) {
    return core::StatusCode::NotInSession;
  }
  if (types_.service(spec.type) == nullptr) {
    return core::StatusCode::UnknownType;
  }
  try {
    auto client = mcu_node_->create_generic_client(spec.name, spec.type);
    if (!client) {
      return core::StatusCode::RosError;
    }
    std::lock_guard<std::mutex> lock(mu_);
    clients_[id] = ClientSlot{std::move(client), spec.type};
    return core::StatusCode::Ok;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "create_generic_client(%s, %s) failed: %s", spec.name.c_str(),
                 spec.type.c_str(), e.what());
    return core::StatusCode::UnknownType;
  }
}

core::StatusCode RclcppBackend::callService(uint8_t id, uint8_t req_id, const uint8_t* cdr,
                                            size_t len) {
  ClientSlot slot;
  {
    std::lock_guard<std::mutex> lock(mu_);
    const auto it = clients_.find(id);
    if (it == clients_.end() || !it->second.client) {
      return core::StatusCode::UnknownEntity;
    }
    slot = it->second;
  }
  auto request = types_.makeRequest(slot.type);
  if (!request || !types_.cdrToRequest(slot.type, cdr, len, request.get())) {
    RCLCPP_ERROR(logger(), "client id=%u req=%u: failed to decode request CDR",
                 static_cast<unsigned>(id), static_cast<unsigned>(req_id));
    deliverServiceFromRos(id, req_id, nullptr, 0U);
    return core::StatusCode::RosError;
  }
  auto pending = std::make_shared<PendingClientCall>();
  pending->id = id;
  pending->req_id = req_id;
  pending->type = slot.type;
  pending->client = slot.client;
  pending->request = request;
  pending->deadline = std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(service_timeout_ms_);
  try {
    auto future_and_id = slot.client->async_send_request(request.get());
    auto holder =
        std::make_shared<std::decay_t<decltype(future_and_id)>>(std::move(future_and_id));
    pending->poll = [holder, pending, this]() -> int {
      if (holder->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto response = holder->get();
        if (!response ||
            !types_.responseToCdr(pending->type, response.get(), &pending->cdr)) {
          return -1;
        }
        return 1;
      }
      if (std::chrono::steady_clock::now() >= pending->deadline) {
        pending->client->remove_pending_request(holder->request_id);
        return -1;
      }
      return 0;
    };
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "client id=%u req=%u: async_send_request failed: %s",
                 static_cast<unsigned>(id), static_cast<unsigned>(req_id), e.what());
    deliverServiceFromRos(id, req_id, nullptr, 0U);
    return core::StatusCode::RosError;
  }
  {
    std::lock_guard<std::mutex> lock(client_call_mu_);
    pending_client_calls_.push_back(std::move(pending));
  }
  return core::StatusCode::Ok;
}

void RclcppBackend::pollServiceClients() {
  std::vector<std::shared_ptr<PendingClientCall>> done;
  {
    std::lock_guard<std::mutex> lock(client_call_mu_);
    std::vector<std::shared_ptr<PendingClientCall>> still;
    still.reserve(pending_client_calls_.size());
    for (auto& pending : pending_client_calls_) {
      if (!pending || !pending->poll) {
        continue;
      }
      const int st = pending->poll();
      if (st == 0) {
        still.push_back(std::move(pending));
        continue;
      }
      done.push_back(std::move(pending));
    }
    pending_client_calls_.swap(still);
  }
  for (const auto& pending : done) {
    if (pending->cdr.empty()) {
      RCLCPP_WARN(logger(), "client id=%u req=%u timed out or failed after %u ms",
                  static_cast<unsigned>(pending->id), static_cast<unsigned>(pending->req_id),
                  static_cast<unsigned>(service_timeout_ms_));
      deliverServiceFromRos(pending->id, pending->req_id, nullptr, 0U);
    } else {
      deliverServiceFromRos(pending->id, pending->req_id, pending->cdr.data(),
                            pending->cdr.size());
    }
  }
}

void RclcppBackend::cancelClientCallsLocked(uint8_t id, bool all) {
  std::vector<std::shared_ptr<PendingClientCall>> still;
  still.reserve(pending_client_calls_.size());
  for (auto& pending : pending_client_calls_) {
    if (!pending) {
      continue;
    }
    if (all || pending->id == id) {
      continue;
    }
    still.push_back(std::move(pending));
  }
  pending_client_calls_.swap(still);
}

core::StatusCode RclcppBackend::publish(uint8_t id, const uint8_t* cdr, size_t len) {
  rclcpp::GenericPublisher::SharedPtr pub;
  {
    std::lock_guard<std::mutex> lock(mu_);
    const auto it = publishers_.find(id);
    if (it == publishers_.end() || !it->second) {
      return core::StatusCode::UnknownEntity;
    }
    pub = it->second;
  }
  try {
    rclcpp::SerializedMessage serialized(len + sizeof(kCdrEncap));
    auto& rcl_msg = serialized.get_rcl_serialized_message();
    std::memcpy(rcl_msg.buffer, kCdrEncap, sizeof(kCdrEncap));
    if (len > 0U && cdr != nullptr) {
      std::memcpy(rcl_msg.buffer + sizeof(kCdrEncap), cdr, len);
    }
    rcl_msg.buffer_length = len + sizeof(kCdrEncap);
    pub->publish(serialized);
    return core::StatusCode::Ok;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "publish id=%u failed: %s", static_cast<unsigned>(id), e.what());
    return core::StatusCode::RosError;
  }
}

core::StatusCode RclcppBackend::createServiceServer(uint8_t id, const host::ServiceSpec& spec) {
#if !defined(ROSDUINO_HAS_GENERIC_SERVICE)
  (void)id;
  RCLCPP_ERROR(logger(),
               "CREATE_SERVICE_SERVER(%s, %s) is unsupported without rclcpp GenericService "
               "(Jazzy has no create_generic_service)",
               spec.name.c_str(), spec.type.c_str());
  return core::StatusCode::Unsupported;
#else
  if (!mcu_node_) {
    return core::StatusCode::NotInSession;
  }
  if (types_.service(spec.type) == nullptr) {
    return core::StatusCode::UnknownType;
  }
  try {
    const std::string type = spec.type;
    auto cb = [this, id, type](const std::shared_ptr<rmw_request_id_t> header,
                               std::shared_ptr<void> request) {
      rclcpp::GenericService::SharedPtr service;
      {
        std::lock_guard<std::mutex> lock(mu_);
        const auto it = services_.find(id);
        if (it != services_.end()) {
          service = it->second;
        }
      }
      auto response = types_.makeResponse(type);
      if (!header || !request || !response || !service) {
        return;
      }
      std::vector<uint8_t> req_cdr;
      if (!types_.requestToCdr(type, request.get(), &req_cdr)) {
        RCLCPP_ERROR(logger(), "service id=%u: failed to encode request CDR",
                     static_cast<unsigned>(id));
        service->send_response(*header, response);
        return;
      }
      auto pending = std::make_shared<InflightRequest>();
      pending->type = type;
      pending->header = header;
      pending->response = std::move(response);
      pending->service = service;
      {
        std::lock_guard<std::mutex> lock(inflight_mu_);
        if (inflight_.count(id) != 0U) {
          RCLCPP_WARN(logger(), "service id=%u busy (one request in flight)",
                      static_cast<unsigned>(id));
          service->send_response(*header, pending->response);
          return;
        }
        pending->req_id = next_req_id_[id]++;
        inflight_[id] = pending;
      }
      deliverServiceToMcu(id, pending->req_id, req_cdr.data(), req_cdr.size());
    };
    auto srv = mcu_node_->create_generic_service(spec.name, spec.type, cb,
                                                 rclcpp::ServicesQoS(), service_cb_group_);
    if (!srv) {
      return core::StatusCode::RosError;
    }
    std::lock_guard<std::mutex> lock(mu_);
    services_[id] = std::move(srv);
    return core::StatusCode::Ok;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger(), "create_generic_service(%s, %s) failed: %s", spec.name.c_str(),
                 spec.type.c_str(), e.what());
    return core::StatusCode::UnknownType;
  }
#endif
}

void RclcppBackend::completeServiceResponse(uint8_t id, uint8_t req_id, const uint8_t* cdr,
                                            size_t len) {
#if !defined(ROSDUINO_HAS_GENERIC_SERVICE)
  (void)id;
  (void)req_id;
  (void)cdr;
  (void)len;
#else
  std::shared_ptr<InflightRequest> pending;
  {
    std::lock_guard<std::mutex> lock(inflight_mu_);
    const auto it = inflight_.find(id);
    if (it == inflight_.end() || !it->second || it->second->req_id != req_id) {
      return;
    }
    pending = it->second;
    inflight_.erase(it);
  }
  if (!pending || !pending->service || !pending->header || !pending->response) {
    return;
  }
  if (cdr != nullptr && len > 0U) {
    if (!types_.cdrToResponse(pending->type, cdr, len, pending->response.get())) {
      RCLCPP_ERROR(logger(), "service id=%u req=%u: failed to decode response CDR",
                   static_cast<unsigned>(id), static_cast<unsigned>(req_id));
    }
  } else {
    RCLCPP_WARN(logger(), "service id=%u req=%u timed out after %u ms",
                static_cast<unsigned>(id), static_cast<unsigned>(req_id),
                static_cast<unsigned>(service_timeout_ms_));
  }
  pending->service->send_response(*pending->header, pending->response);
#endif
}

void RclcppBackend::failInflight(uint8_t id) {
  std::shared_ptr<InflightRequest> pending;
  {
    std::lock_guard<std::mutex> lock(inflight_mu_);
    const auto it = inflight_.find(id);
    if (it == inflight_.end()) {
      return;
    }
    pending = it->second;
    inflight_.erase(it);
  }
#if defined(ROSDUINO_HAS_GENERIC_SERVICE)
  if (pending && pending->service && pending->header && pending->response) {
    pending->service->send_response(*pending->header, pending->response);
  }
#else
  (void)pending;
#endif
}

void RclcppBackend::failInflightLocked() {
  std::vector<std::shared_ptr<InflightRequest>> pending;
  pending.reserve(inflight_.size());
  for (auto& item : inflight_) {
    pending.push_back(item.second);
  }
  inflight_.clear();
#if defined(ROSDUINO_HAS_GENERIC_SERVICE)
  for (auto& item : pending) {
    if (item && item->service && item->header && item->response) {
      item->service->send_response(*item->header, item->response);
    }
  }
#endif
}

void RclcppBackend::destroyEntity(uint8_t id) {
  failInflight(id);
  {
    std::lock_guard<std::mutex> lock(client_call_mu_);
    cancelClientCallsLocked(id, false);
  }
  std::lock_guard<std::mutex> lock(mu_);
  publishers_.erase(id);
  subscriptions_.erase(id);
  clients_.erase(id);
#if defined(ROSDUINO_HAS_GENERIC_SERVICE)
  services_.erase(id);
#endif
}

void RclcppBackend::destroyAllEntities() {
  {
    std::lock_guard<std::mutex> lock(inflight_mu_);
    failInflightLocked();
  }
  {
    std::lock_guard<std::mutex> lock(client_call_mu_);
    cancelClientCallsLocked(0U, true);
  }
  std::lock_guard<std::mutex> lock(mu_);
  publishers_.clear();
  subscriptions_.clear();
  clients_.clear();
#if defined(ROSDUINO_HAS_GENERIC_SERVICE)
  services_.clear();
#endif
}

void RclcppBackend::log(core::LogLevel level, const std::string& text) {
  const char* msg = text.c_str();
  switch (level) {
    case core::LogLevel::Debug:
      RCLCPP_DEBUG(logger(), "%s", msg);
      break;
    case core::LogLevel::Info:
      RCLCPP_INFO(logger(), "%s", msg);
      break;
    case core::LogLevel::Warn:
      RCLCPP_WARN(logger(), "%s", msg);
      break;
    case core::LogLevel::Error:
      RCLCPP_ERROR(logger(), "%s", msg);
      break;
    case core::LogLevel::Fatal:
      RCLCPP_FATAL(logger(), "%s", msg);
      break;
    default:
      RCLCPP_INFO(logger(), "%s", msg);
      break;
  }
}

}  // namespace agent
}  // namespace rosduino
