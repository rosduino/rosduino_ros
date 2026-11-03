// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_WRITE_QUEUE_H_
#define ROSDUINO_HOST_WRITE_QUEUE_H_

#include <cstdint>
#include <mutex>
#include <vector>

#include "rosduino/core/frame.h"

namespace rosduino {
namespace host {

struct QueuedFrame {
  uint8_t type = 0U;
  uint8_t seq = 0U;
  std::vector<uint8_t> body;
};

/**
 * @brief Single-writer outbound queue. ROS callbacks enqueue; the I/O thread flushes.
 */
class WriteQueue {
 public:
  void push(uint8_t type, uint8_t seq, const uint8_t* body, size_t len) {
    QueuedFrame frame;
    frame.type = type;
    frame.seq = seq;
    if (body != nullptr && len > 0U) {
      frame.body.assign(body, body + len);
    }
    std::lock_guard<std::mutex> lock(mu_);
    q_.push_back(std::move(frame));
  }

  template <typename Sink>
  size_t flush(core::FrameWriter& writer, size_t peer_rx_max, Sink& sink) {
    std::vector<QueuedFrame> local;
    {
      std::lock_guard<std::mutex> lock(mu_);
      local.swap(q_);
    }
    size_t sent = 0U;
    for (const QueuedFrame& frame : local) {
      const uint8_t* body = frame.body.empty() ? nullptr : frame.body.data();
      if (writer.writeFrame(frame.type, frame.seq, body, frame.body.size(), sink, peer_rx_max)) {
        ++sent;
      }
    }
    return sent;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mu_);
    return q_.size();
  }

 private:
  mutable std::mutex mu_;
  std::vector<QueuedFrame> q_;
};

}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_WRITE_QUEUE_H_
