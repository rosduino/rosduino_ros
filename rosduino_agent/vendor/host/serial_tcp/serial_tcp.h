// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_SERIAL_TCP_H_
#define ROSDUINO_HOST_SERIAL_TCP_H_

#include <atomic>
#include <cstdint>
#include <string>

#include "transport/serial_transport.h"

namespace rosduino {
namespace host {

/**
 * @brief Transparent serial ↔ TCP byte pipe for an agent that cannot open the port itself.
 *
 * Keeps the serial port open (so Arduino does not reset on agent reconnect).
 * At most one TCP client; extra connections are closed. USB unplug: wait and
 * reopen the serial path.
 */
class SerialTcpBridge {
 public:
  struct Config {
    std::string serial_path;
    uint32_t baud = 115200U;
    std::string listen_host = "127.0.0.1";
    uint16_t listen_port = 5760U;
    bool reset_on_connect = false;
    bool serial_dtr = true;
  };

  explicit SerialTcpBridge(Config cfg);
  ~SerialTcpBridge();

  SerialTcpBridge(const SerialTcpBridge&) = delete;
  SerialTcpBridge& operator=(const SerialTcpBridge&) = delete;

  bool start();
  void run(std::atomic<bool>& running);
  uint16_t boundPort() const { return bound_port_; }
  const char* lastError() const { return err_.c_str(); }
  std::string describe() const;

 private:
  void closeListen();
  void closeClient();
  bool bindListen();
  void acceptClient();
  void pulseDtr();
  void pumpSerialToClient();
  void pumpClientToSerial();

  Config cfg_;
  SerialTransport serial_;
  int listen_fd_;
  int client_fd_;
  uint16_t bound_port_;
  std::string err_;
};

}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_SERIAL_TCP_H_
