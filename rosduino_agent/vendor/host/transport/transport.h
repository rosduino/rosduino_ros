// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_TRANSPORT_H_
#define ROSDUINO_HOST_TRANSPORT_H_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <sys/types.h>

namespace rosduino {
namespace host {

/**
 * @brief Byte-stream transport used by the agent and tests (serial or TCP).
 *
 * Not thread-safe: the bridge serializes writes under a mutex and reads on one
 * thread.
 */
class Transport {
 public:
  virtual ~Transport() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  /**
   * @brief Open if closed. TCP waits `reconnect_interval` after a failed
   *        attempt or drop; serial retries immediately.
   */
  virtual bool ensureOpen() = 0;

  /** @brief Wait until the fd is readable, or `timeout_ms` elapses. */
  virtual bool waitReadable(int timeout_ms) = 0;

  /**
   * @return Bytes read; 0 if no data; -1 if the connection was lost.
   */
  virtual ssize_t read(uint8_t* buf, size_t len) = 0;

  /**
   * @brief Write all `len` bytes, retrying short writes.
   * @return `len` on success, -1 on disconnect.
   */
  virtual ssize_t write(const uint8_t* buf, size_t len) = 0;

  virtual int nativeFd() const = 0;
  virtual std::string describe() const = 0;
  virtual const char* lastError() const = 0;
};

struct TransportConfig {
  std::string endpoint;  // device path, or `tcp://host:port`
  uint32_t baud = 115200U;
  bool dtr = true;  // false: leave modem bits alone (PTY / no Arduino reset)
  std::chrono::milliseconds reconnect_interval{1000};
};

/**
 * @brief Build a serial transport for a device path, or a TCP client for
 *        `tcp://host:port`.
 */
std::unique_ptr<Transport> makeTransport(const TransportConfig& cfg);

}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_TRANSPORT_H_
