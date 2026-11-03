// SPDX-License-Identifier: Apache-2.0
#include "type_registry.hpp"

#include <cstdlib>
#include <cstring>
#include <exception>

#include "rclcpp/serialized_message.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

namespace rosduino {
namespace agent {
namespace {

constexpr uint8_t kCdrEncap[4] = {0x00, 0x01, 0x00, 0x00};

template <typename Ts>
uint64_t readHash(const Ts* ts) {
  uint64_t hash = 0ULL;
  if (ts != nullptr && ts->get_type_hash_func != nullptr) {
    const rosidl_type_hash_t* h = ts->get_type_hash_func(ts);
    if (h != nullptr && h->version == 1) {
      std::memcpy(&hash, h->value, sizeof(hash));
    }
  }
  return hash;
}

}  // namespace

std::shared_ptr<void> TypeRegistry::allocate(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members) {
  if (members == nullptr || members->size_of_ == 0U) {
    return nullptr;
  }
  void* mem = std::malloc(members->size_of_);
  if (mem == nullptr) {
    return nullptr;
  }
  members->init_function(mem, rosidl_runtime_cpp::MessageInitialization::ALL);
  return std::shared_ptr<void>(mem, [members](void* p) {
    members->fini_function(p);
    std::free(p);
  });
}

uint64_t TypeRegistry::hashFromTypeSupport(const rosidl_message_type_support_t* ts) {
  return readHash(ts);
}

uint64_t TypeRegistry::hashFromTypeSupport(const rosidl_service_type_support_t* ts) {
  return readHash(ts);
}

bool TypeRegistry::payloadToSerialized(const uint8_t* cdr, size_t len,
                                       rclcpp::SerializedMessage* out) {
  if (out == nullptr) {
    return false;
  }
  *out = rclcpp::SerializedMessage(len + sizeof(kCdrEncap));
  auto& rcl_msg = out->get_rcl_serialized_message();
  if (rcl_msg.buffer == nullptr || rcl_msg.buffer_capacity < len + sizeof(kCdrEncap)) {
    return false;
  }
  std::memcpy(rcl_msg.buffer, kCdrEncap, sizeof(kCdrEncap));
  if (len > 0U && cdr != nullptr) {
    std::memcpy(rcl_msg.buffer + sizeof(kCdrEncap), cdr, len);
  }
  rcl_msg.buffer_length = len + sizeof(kCdrEncap);
  return true;
}

bool TypeRegistry::serializedToPayload(const rclcpp::SerializedMessage& in,
                                       std::vector<uint8_t>* out) {
  if (out == nullptr) {
    return false;
  }
  const auto& rcl_msg = in.get_rcl_serialized_message();
  if (rcl_msg.buffer == nullptr || rcl_msg.buffer_length < sizeof(kCdrEncap)) {
    return false;
  }
  if ((rcl_msg.buffer[1] & 0x01U) != 0x01U) {
    return false;
  }
  out->assign(rcl_msg.buffer + sizeof(kCdrEncap), rcl_msg.buffer + rcl_msg.buffer_length);
  return true;
}

bool TypeRegistry::cdrToMem(const rclcpp::SerializationBase& ser, const uint8_t* cdr, size_t len,
                            void* msg) {
  if (msg == nullptr) {
    return false;
  }
  try {
    rclcpp::SerializedMessage serialized;
    if (!payloadToSerialized(cdr, len, &serialized)) {
      return false;
    }
    ser.deserialize_message(&serialized, msg);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool TypeRegistry::memToCdr(const rclcpp::SerializationBase& ser, const void* msg,
                            std::vector<uint8_t>* out) {
  if (msg == nullptr || out == nullptr) {
    return false;
  }
  try {
    rclcpp::SerializedMessage serialized;
    ser.serialize_message(msg, &serialized);
    return serializedToPayload(serialized, out);
  } catch (const std::exception&) {
    return false;
  }
}

const TypeRegistry::MessageCodec* TypeRegistry::loadMessage(const std::string& type) const {
  std::lock_guard<std::mutex> lock(mu_);
  const auto it = messages_.find(type);
  if (it != messages_.end()) {
    return it->second.get();
  }
  try {
    auto codec = std::make_unique<MessageCodec>();
    codec->cpp_lib = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    if (!codec->cpp_lib) {
      return nullptr;
    }
    codec->ts_cpp =
        rclcpp::get_message_typesupport_handle(type, "rosidl_typesupport_cpp", *codec->cpp_lib);
    if (codec->ts_cpp == nullptr) {
      return nullptr;
    }
    codec->serializer = std::make_unique<rclcpp::SerializationBase>(codec->ts_cpp);
    codec->intro_lib = rclcpp::get_typesupport_library(
        type, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    if (!codec->intro_lib) {
      return nullptr;
    }
    const rosidl_message_type_support_t* ts_intro = rclcpp::get_message_typesupport_handle(
        type, rosidl_typesupport_introspection_cpp::typesupport_identifier, *codec->intro_lib);
    if (ts_intro == nullptr) {
      return nullptr;
    }
    codec->members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(ts_intro->data);
    codec->hash = hashFromTypeSupport(codec->ts_cpp);
    const MessageCodec* raw = codec.get();
    messages_[type] = std::move(codec);
    return raw;
  } catch (const std::exception&) {
    return nullptr;
  }
}

const TypeRegistry::ServiceCodec* TypeRegistry::loadService(const std::string& type) const {
  std::lock_guard<std::mutex> lock(mu_);
  const auto it = services_.find(type);
  if (it != services_.end()) {
    return it->second.get();
  }
  try {
    auto codec = std::make_unique<ServiceCodec>();
    codec->cpp_lib = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    if (!codec->cpp_lib) {
      return nullptr;
    }
    codec->ts_cpp =
        rclcpp::get_service_typesupport_handle(type, "rosidl_typesupport_cpp", *codec->cpp_lib);
    if (codec->ts_cpp == nullptr) {
      return nullptr;
    }
    codec->request = std::make_unique<rclcpp::SerializationBase>(codec->ts_cpp->request_typesupport);
    codec->response =
        std::make_unique<rclcpp::SerializationBase>(codec->ts_cpp->response_typesupport);
    codec->intro_lib = rclcpp::get_typesupport_library(
        type, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    if (!codec->intro_lib) {
      return nullptr;
    }
    const rosidl_service_type_support_t* ts_intro = rclcpp::get_service_typesupport_handle(
        type, rosidl_typesupport_introspection_cpp::typesupport_identifier, *codec->intro_lib);
    if (ts_intro == nullptr) {
      return nullptr;
    }
    codec->members =
        static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers*>(ts_intro->data);
    codec->hash = hashFromTypeSupport(codec->ts_cpp);
    const ServiceCodec* raw = codec.get();
    services_[type] = std::move(codec);
    return raw;
  } catch (const std::exception&) {
    return nullptr;
  }
}

const TypeRegistry::MessageCodec* TypeRegistry::message(const std::string& type) const {
  return loadMessage(type);
}

const TypeRegistry::ServiceCodec* TypeRegistry::service(const std::string& type) const {
  return loadService(type);
}

std::optional<uint64_t> TypeRegistry::messageHash(const std::string& type) const {
  const MessageCodec* codec = loadMessage(type);
  if (codec == nullptr) {
    return std::nullopt;
  }
  return codec->hash;
}

std::optional<uint64_t> TypeRegistry::serviceHash(const std::string& type) const {
  const ServiceCodec* codec = loadService(type);
  if (codec == nullptr) {
    return std::nullopt;
  }
  return codec->hash;
}

std::shared_ptr<void> TypeRegistry::makeMessage(const std::string& type) const {
  const MessageCodec* codec = loadMessage(type);
  if (codec == nullptr) {
    return nullptr;
  }
  return allocate(codec->members);
}

std::shared_ptr<void> TypeRegistry::makeRequest(const std::string& type) const {
  const ServiceCodec* codec = loadService(type);
  if (codec == nullptr || codec->members == nullptr) {
    return nullptr;
  }
  return allocate(codec->members->request_members_);
}

std::shared_ptr<void> TypeRegistry::makeResponse(const std::string& type) const {
  const ServiceCodec* codec = loadService(type);
  if (codec == nullptr || codec->members == nullptr) {
    return nullptr;
  }
  return allocate(codec->members->response_members_);
}

bool TypeRegistry::cdrToMessage(const std::string& type, const uint8_t* cdr, size_t len,
                                void* msg) const {
  const MessageCodec* codec = loadMessage(type);
  if (codec == nullptr || codec->serializer == nullptr) {
    return false;
  }
  return cdrToMem(*codec->serializer, cdr, len, msg);
}

bool TypeRegistry::messageToCdr(const std::string& type, const void* msg,
                                std::vector<uint8_t>* out) const {
  const MessageCodec* codec = loadMessage(type);
  if (codec == nullptr || codec->serializer == nullptr) {
    return false;
  }
  return memToCdr(*codec->serializer, msg, out);
}

bool TypeRegistry::cdrToRequest(const std::string& type, const uint8_t* cdr, size_t len,
                                void* msg) const {
  const ServiceCodec* codec = loadService(type);
  if (codec == nullptr || codec->request == nullptr) {
    return false;
  }
  return cdrToMem(*codec->request, cdr, len, msg);
}

bool TypeRegistry::requestToCdr(const std::string& type, const void* msg,
                                std::vector<uint8_t>* out) const {
  const ServiceCodec* codec = loadService(type);
  if (codec == nullptr || codec->request == nullptr) {
    return false;
  }
  return memToCdr(*codec->request, msg, out);
}

bool TypeRegistry::cdrToResponse(const std::string& type, const uint8_t* cdr, size_t len,
                                 void* msg) const {
  const ServiceCodec* codec = loadService(type);
  if (codec == nullptr || codec->response == nullptr) {
    return false;
  }
  return cdrToMem(*codec->response, cdr, len, msg);
}

bool TypeRegistry::responseToCdr(const std::string& type, const void* msg,
                                 std::vector<uint8_t>* out) const {
  const ServiceCodec* codec = loadService(type);
  if (codec == nullptr || codec->response == nullptr) {
    return false;
  }
  return memToCdr(*codec->response, msg, out);
}

}  // namespace agent
}  // namespace rosduino
