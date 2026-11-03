// SPDX-License-Identifier: Apache-2.0
#include <cstdint>
#include <cstring>
#include <vector>

#include "gtest/gtest.h"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "type_registry.hpp"

namespace {

uint64_t le_hash(const uint8_t bytes[8]) {
  uint64_t h = 0ULL;
  std::memcpy(&h, bytes, sizeof(h));
  return h;
}

}  // namespace

TEST(TypeRegistry, MessageHashMatchesRihsPrefix) {
  rosduino::agent::TypeRegistry types;
  const auto hash = types.messageHash("std_msgs/msg/Int32");
  ASSERT_TRUE(hash.has_value());
  const uint8_t expected[8] = {0xb6U, 0x57U, 0x8dU, 0xedU, 0x3cU, 0x58U, 0xc6U, 0x26U};
  EXPECT_EQ(*hash, le_hash(expected));
}

TEST(TypeRegistry, ServiceHashMatchesRihsPrefix) {
  rosduino::agent::TypeRegistry types;
  const auto hash = types.serviceHash("std_srvs/srv/SetBool");
  ASSERT_TRUE(hash.has_value());
  const uint8_t expected[8] = {0xabU, 0xe9U, 0xe4U, 0xbbU, 0x6bU, 0x41U, 0xb4U, 0x0eU};
  EXPECT_EQ(*hash, le_hash(expected));
}

TEST(TypeRegistry, UnknownTypeReturnsNullopt) {
  rosduino::agent::TypeRegistry types;
  EXPECT_FALSE(types.messageHash("not_a_pkg/msg/Nope").has_value());
  EXPECT_FALSE(types.serviceHash("not_a_pkg/srv/Nope").has_value());
  EXPECT_EQ(types.message("not_a_pkg/msg/Nope"), nullptr);
  EXPECT_EQ(types.service("not_a_pkg/srv/Nope"), nullptr);
}

TEST(TypeRegistry, CachesCodecByTypeName) {
  rosduino::agent::TypeRegistry types;
  const auto* a = types.message("std_msgs/msg/Int32");
  const auto* b = types.message("std_msgs/msg/Int32");
  ASSERT_NE(a, nullptr);
  EXPECT_EQ(a, b);
  const auto* s1 = types.service("std_srvs/srv/SetBool");
  const auto* s2 = types.service("std_srvs/srv/SetBool");
  ASSERT_NE(s1, nullptr);
  EXPECT_EQ(s1, s2);
}

TEST(TypeRegistry, Int32CdrRoundTrip) {
  rosduino::agent::TypeRegistry types;
  const uint8_t payload[] = {0xd6, 0xff, 0xff, 0xff};
  auto msg = types.makeMessage("std_msgs/msg/Int32");
  ASSERT_NE(msg, nullptr);
  ASSERT_TRUE(types.cdrToMessage("std_msgs/msg/Int32", payload, sizeof(payload), msg.get()));
  EXPECT_EQ(static_cast<const std_msgs::msg::Int32*>(msg.get())->data, -42);

  std::vector<uint8_t> out;
  ASSERT_TRUE(types.messageToCdr("std_msgs/msg/Int32", msg.get(), &out));
  ASSERT_GE(out.size(), sizeof(payload));
  EXPECT_EQ(0, std::memcmp(out.data(), payload, sizeof(payload)));
}

TEST(TypeRegistry, SetBoolRequestResponseCdr) {
  rosduino::agent::TypeRegistry types;
  const uint8_t req_cdr[] = {0x01};
  auto req = types.makeRequest("std_srvs/srv/SetBool");
  ASSERT_NE(req, nullptr);
  ASSERT_TRUE(types.cdrToRequest("std_srvs/srv/SetBool", req_cdr, sizeof(req_cdr), req.get()));
  EXPECT_TRUE(static_cast<const std_srvs::srv::SetBool::Request*>(req.get())->data);

  std::vector<uint8_t> req_out;
  ASSERT_TRUE(types.requestToCdr("std_srvs/srv/SetBool", req.get(), &req_out));
  ASSERT_FALSE(req_out.empty());
  EXPECT_EQ(req_out[0], 0x01);

  auto res = types.makeResponse("std_srvs/srv/SetBool");
  ASSERT_NE(res, nullptr);
  auto* typed = static_cast<std_srvs::srv::SetBool::Response*>(res.get());
  typed->success = true;
  typed->message = "ok";
  std::vector<uint8_t> res_out;
  ASSERT_TRUE(types.responseToCdr("std_srvs/srv/SetBool", res.get(), &res_out));
  auto res2 = types.makeResponse("std_srvs/srv/SetBool");
  ASSERT_TRUE(types.cdrToResponse("std_srvs/srv/SetBool", res_out.data(), res_out.size(),
                                  res2.get()));
  const auto* typed2 = static_cast<const std_srvs::srv::SetBool::Response*>(res2.get());
  EXPECT_TRUE(typed2->success);
  EXPECT_EQ(typed2->message, "ok");
}

TEST(TypeRegistry, AddTwoIntsRequestCdr) {
  rosduino::agent::TypeRegistry types;
  const uint8_t req_cdr[] = {
      0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00,
  };
  auto req = types.makeRequest("example_interfaces/srv/AddTwoInts");
  ASSERT_NE(req, nullptr);
  ASSERT_TRUE(
      types.cdrToRequest("example_interfaces/srv/AddTwoInts", req_cdr, sizeof(req_cdr), req.get()));
  std::vector<uint8_t> out;
  ASSERT_TRUE(types.requestToCdr("example_interfaces/srv/AddTwoInts", req.get(), &out));
  ASSERT_GE(out.size(), sizeof(req_cdr));
  EXPECT_EQ(0, std::memcmp(out.data(), req_cdr, sizeof(req_cdr)));
}
