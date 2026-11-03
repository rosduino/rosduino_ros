// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_ROS_BACKEND_H_
#define ROSDUINO_HOST_ROS_BACKEND_H_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "rosduino/core/commands.h"

namespace rosduino {
namespace host {

struct TopicSpec {
  std::string name;
  std::string type;
  uint64_t hash = 0ULL;
  uint8_t qos = 0U;
  uint8_t depth = 0U;
};

struct ServiceSpec {
  std::string name;
  std::string type;
  uint64_t hash = 0ULL;
};

/**
 * @brief ROS 2 operations used by the bridge. No ROS headers in `host/`.
 *
 * Publishers, subscriptions, MCU service servers, and MCU service clients.
 */
class RosBackend {
 public:
  using IncomingHandler = std::function<void(uint8_t id, const uint8_t* cdr, size_t len)>;
  using ServiceToMcuHandler =
      std::function<void(uint8_t id, uint8_t req_id, const uint8_t* cdr, size_t len)>;
  using ServiceFromRosHandler =
      std::function<void(uint8_t id, uint8_t req_id, const uint8_t* cdr, size_t len)>;

  virtual ~RosBackend() = default;

  void setIncomingHandler(IncomingHandler fn) { incoming_ = std::move(fn); }
  void setServiceToMcuHandler(ServiceToMcuHandler fn) { service_to_mcu_ = std::move(fn); }
  void setServiceFromRosHandler(ServiceFromRosHandler fn) { service_from_ros_ = std::move(fn); }

  /**
   * @brief Create the MCU's ROS node. On a graph name clash, add `_2`, `_3`, …
   * @return Actual node name, or empty string on failure.
   */
  virtual std::string createNode(const std::string& name, const std::string& ns) = 0;
  virtual void destroyNode() = 0;

  /** @return Type hash in the ROS environment, or nullopt if the type is unknown. */
  virtual std::optional<uint64_t> lookupTypeHash(const std::string& type) const = 0;

  virtual core::StatusCode createPublisher(uint8_t id, const TopicSpec& spec) = 0;
  virtual core::StatusCode createSubscriber(uint8_t id, const TopicSpec& spec) = 0;
  virtual core::StatusCode createServiceServer(uint8_t id, const ServiceSpec& spec) = 0;
  virtual core::StatusCode createServiceClient(uint8_t id, const ServiceSpec& spec) = 0;
  virtual core::StatusCode publish(uint8_t id, const uint8_t* cdr, size_t len) = 0;
  virtual core::StatusCode callService(uint8_t id, uint8_t req_id, const uint8_t* cdr,
                                       size_t len) = 0;
  virtual void completeServiceResponse(uint8_t id, uint8_t req_id, const uint8_t* cdr,
                                       size_t len) = 0;
  virtual void pollServiceClients() {}
  virtual void destroyEntity(uint8_t id) = 0;
  virtual void destroyAllEntities() = 0;
  virtual void log(core::LogLevel level, const std::string& text) = 0;

 protected:
  void deliverIncoming(uint8_t id, const uint8_t* cdr, size_t len) const {
    if (incoming_) {
      incoming_(id, cdr, len);
    }
  }

  void deliverServiceToMcu(uint8_t id, uint8_t req_id, const uint8_t* cdr, size_t len) const {
    if (service_to_mcu_) {
      service_to_mcu_(id, req_id, cdr, len);
    }
  }

  void deliverServiceFromRos(uint8_t id, uint8_t req_id, const uint8_t* cdr, size_t len) const {
    if (service_from_ros_) {
      service_from_ros_(id, req_id, cdr, len);
    }
  }

 private:
  IncomingHandler incoming_;
  ServiceToMcuHandler service_to_mcu_;
  ServiceFromRosHandler service_from_ros_;
};

}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_ROS_BACKEND_H_
