// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_AGENT_TYPE_REGISTRY_HPP_
#define ROSDUINO_AGENT_TYPE_REGISTRY_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/serialization.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

namespace rosduino {
namespace agent {

/**
 * @brief Runtime typesupport lookup, RIHS01 hash, and CDR ↔ `void*` conversion.
 *
 * Topics keep using generic CDR as-is. Services need a heap message that
 * `GenericClient` / `GenericService` understand; this registry builds that
 * from MCU payload bytes (XCDR1 LE without the 4-byte encapsulation header).
 */
class TypeRegistry {
 public:
  struct MessageCodec {
    std::shared_ptr<rcpputils::SharedLibrary> cpp_lib;
    std::shared_ptr<rcpputils::SharedLibrary> intro_lib;
    const rosidl_message_type_support_t* ts_cpp = nullptr;
    const rosidl_typesupport_introspection_cpp::MessageMembers* members = nullptr;
    std::unique_ptr<rclcpp::SerializationBase> serializer;
    uint64_t hash = 0ULL;
  };

  struct ServiceCodec {
    std::shared_ptr<rcpputils::SharedLibrary> cpp_lib;
    std::shared_ptr<rcpputils::SharedLibrary> intro_lib;
    const rosidl_service_type_support_t* ts_cpp = nullptr;
    const rosidl_typesupport_introspection_cpp::ServiceMembers* members = nullptr;
    std::unique_ptr<rclcpp::SerializationBase> request;
    std::unique_ptr<rclcpp::SerializationBase> response;
    uint64_t hash = 0ULL;
  };

  std::optional<uint64_t> messageHash(const std::string& type) const;
  std::optional<uint64_t> serviceHash(const std::string& type) const;

  const MessageCodec* message(const std::string& type) const;
  const ServiceCodec* service(const std::string& type) const;

  std::shared_ptr<void> makeMessage(const std::string& type) const;
  std::shared_ptr<void> makeRequest(const std::string& type) const;
  std::shared_ptr<void> makeResponse(const std::string& type) const;

  bool cdrToMessage(const std::string& type, const uint8_t* cdr, size_t len, void* msg) const;
  bool messageToCdr(const std::string& type, const void* msg, std::vector<uint8_t>* out) const;
  bool cdrToRequest(const std::string& type, const uint8_t* cdr, size_t len, void* msg) const;
  bool requestToCdr(const std::string& type, const void* msg, std::vector<uint8_t>* out) const;
  bool cdrToResponse(const std::string& type, const uint8_t* cdr, size_t len, void* msg) const;
  bool responseToCdr(const std::string& type, const void* msg, std::vector<uint8_t>* out) const;

  static std::shared_ptr<void> allocate(
      const rosidl_typesupport_introspection_cpp::MessageMembers* members);

 private:
  const MessageCodec* loadMessage(const std::string& type) const;
  const ServiceCodec* loadService(const std::string& type) const;

  static uint64_t hashFromTypeSupport(const rosidl_message_type_support_t* ts);
  static uint64_t hashFromTypeSupport(const rosidl_service_type_support_t* ts);
  static bool payloadToSerialized(const uint8_t* cdr, size_t len,
                                  rclcpp::SerializedMessage* out);
  static bool serializedToPayload(const rclcpp::SerializedMessage& in,
                                  std::vector<uint8_t>* out);
  static bool cdrToMem(const rclcpp::SerializationBase& ser, const uint8_t* cdr, size_t len,
                       void* msg);
  static bool memToCdr(const rclcpp::SerializationBase& ser, const void* msg,
                       std::vector<uint8_t>* out);

  mutable std::mutex mu_;
  mutable std::map<std::string, std::unique_ptr<MessageCodec>> messages_;
  mutable std::map<std::string, std::unique_ptr<ServiceCodec>> services_;
};

}  // namespace agent
}  // namespace rosduino

#endif  // ROSDUINO_AGENT_TYPE_REGISTRY_HPP_
