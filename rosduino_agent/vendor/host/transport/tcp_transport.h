// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_TCP_TRANSPORT_H_
#define ROSDUINO_HOST_TCP_TRANSPORT_H_

#include <chrono>
#include <cstdint>
#include <string>

#include "transport/transport.h"

namespace rosduino {
namespace host {

class TcpClientTransport final : public Transport {
 public:
  TcpClientTransport(std::string host, uint16_t port,
                     std::chrono::milliseconds reconnect_interval = std::chrono::milliseconds(1000));
  ~TcpClientTransport() override;

  TcpClientTransport(const TcpClientTransport&) = delete;
  TcpClientTransport& operator=(const TcpClientTransport&) = delete;

  bool open() override;
  void close() override;
  bool isOpen() const override;
  bool ensureOpen() override;
  bool waitReadable(int timeout_ms) override;
  ssize_t read(uint8_t* buf, size_t len) override;
  ssize_t write(const uint8_t* buf, size_t len) override;
  int nativeFd() const override;
  std::string describe() const override;
  const char* lastError() const override;

 private:
  void drop();

  std::string host_;
  uint16_t port_;
  std::chrono::milliseconds reconnect_interval_;
  std::chrono::steady_clock::time_point last_attempt_{};
  bool throttle_reconnect_;
  int fd_;
  std::string err_;
};

}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_TCP_TRANSPORT_H_
