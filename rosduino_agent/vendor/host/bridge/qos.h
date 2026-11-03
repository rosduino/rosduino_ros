// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_QOS_H_
#define ROSDUINO_HOST_QOS_H_

#include <cstdint>

#include "rosduino/core/commands.h"

namespace rosduino {
namespace host {

struct DecodedTopicQos {
  bool reliable = true;
  bool transient_local = false;
  uint8_t history_depth = 10U;
};

/** Map protocol `qos:u8` / `depth:u8` (`docs/protocol.md` §3.4) to ROS 2 KEEP_LAST QoS. */
inline DecodedTopicQos decodeTopicQos(uint8_t qos, uint8_t depth) {
  DecodedTopicQos out;
  out.reliable = (qos & core::kQosReliable) != 0U;
  out.transient_local = (qos & core::kQosTransientLocal) != 0U;
  out.history_depth = (depth == 0U) ? 1U : depth;
  return out;
}

}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_QOS_H_
