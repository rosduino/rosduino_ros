// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_SERIAL_TRANSPORT_H_
#define ROSDUINO_HOST_SERIAL_TRANSPORT_H_

#include <cstdint>
#include <string>

#include "transport/transport.h"

namespace rosduino {
namespace host {

class SerialTransport final : public Transport {
 public:
  SerialTransport(std::string path, uint32_t baud, bool dtr = true);
  ~SerialTransport() override;

  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;

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
  bool configureTermios();
  void applyDtr();
  void drop();

  std::string path_;
  uint32_t baud_;
  bool dtr_;
  int fd_;
  std::string err_;
};

}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_SERIAL_TRANSPORT_H_
