// SPDX-License-Identifier: Apache-2.0
#include "Arduino.h"

#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;
const Clock::time_point kStart = Clock::now();

constexpr uint8_t kPinCount = 32U;
uint8_t g_pin_mode[kPinCount] = {};
uint8_t g_pin_value[kPinCount] = {};

int g_analog_default = 512;

}  // namespace

uint32_t millis() {
  const auto d = Clock::now() - kStart;
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(d).count());
}

uint32_t micros() {
  const auto d = Clock::now() - kStart;
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(d).count());
}

void delay(unsigned long ms) {
  const unsigned int usec = static_cast<unsigned int>(ms * 1000UL);
  ::usleep(usec == 0U ? 1U : usec);
}

void pinMode(uint8_t pin, uint8_t mode) {
  if (pin < kPinCount) {
    g_pin_mode[pin] = mode;
  }
}

void digitalWrite(uint8_t pin, uint8_t val) {
  if (pin < kPinCount) {
    g_pin_value[pin] = (val != 0U) ? HIGH : LOW;
  }
}

int digitalRead(uint8_t pin) {
  if (pin >= kPinCount) {
    return LOW;
  }
  return g_pin_value[pin];
}

int analogRead(uint8_t /*pin*/) { return g_analog_default; }

HardwareSerial::HardwareSerial()
    : fd_(-1),
      baud_(0UL),
      tx_next_us_(0U),
      rx_last_us_(0U),
      rx_credit_us_(0U),
      overruns_(0U),
      rx_head_(0U),
      rx_tail_(0U),
      rx_count_(0U) {}

uint32_t HardwareSerial::byteTimeUs() const {
  if (baud_ == 0UL) {
    return 0U;
  }
  return (10000000UL + (static_cast<uint32_t>(baud_) / 2UL)) / static_cast<uint32_t>(baud_);
}

void HardwareSerial::begin(unsigned long baud) {
  baud_ = baud;
  tx_next_us_ = 0U;
  rx_last_us_ = micros();
  rx_credit_us_ = 0U;
  overruns_ = 0U;
  rx_head_ = 0U;
  rx_tail_ = 0U;
  rx_count_ = 0U;
}

void HardwareSerial::waitTxSlot() {
  const uint32_t byte_us = byteTimeUs();
  if (byte_us == 0U) {
    return;
  }
  uint32_t now = micros();
  if (tx_next_us_ == 0U) {
    tx_next_us_ = now;
  }
  while (static_cast<int32_t>(tx_next_us_ - now) > 0) {
    const int32_t remain = static_cast<int32_t>(tx_next_us_ - now);
    if (remain > 2000) {
      ::usleep(static_cast<unsigned int>(remain - 1000));
    }
    now = micros();
  }
  now = micros();
  tx_next_us_ = now + byte_us;
}

bool HardwareSerial::fifoPush(uint8_t b) {
  if (rx_count_ >= kRxFifo) {
    ++overruns_;
    return false;
  }
  rx_fifo_[rx_tail_] = b;
  rx_tail_ = static_cast<uint8_t>((rx_tail_ + 1U) % kRxFifo);
  ++rx_count_;
  return true;
}

int HardwareSerial::fifoPop() {
  if (rx_count_ == 0U) {
    return -1;
  }
  const int value = static_cast<int>(rx_fifo_[rx_head_]);
  rx_head_ = static_cast<uint8_t>((rx_head_ + 1U) % kRxFifo);
  --rx_count_;
  return value;
}

void HardwareSerial::pumpRx() {
  if (fd_ < 0 || baud_ == 0UL) {
    return;
  }
  const uint32_t byte_us = byteTimeUs();
  const uint32_t now = micros();
  if (rx_last_us_ == 0U) {
    rx_last_us_ = now;
  } else {
    const uint32_t elapsed = now - rx_last_us_;
    rx_credit_us_ += elapsed;
    rx_last_us_ = now;
  }
  while (rx_credit_us_ >= byte_us) {
    uint8_t b = 0;
    const ssize_t n = ::read(fd_, &b, 1);
    if (n == 1) {
      (void)fifoPush(b);
      rx_credit_us_ -= byte_us;
    } else {
      if (n == 0) {
        close();
      }
      rx_credit_us_ = 0U;
      break;
    }
  }
}

void HardwareSerial::attach(int fd) {
  close();
  fd_ = fd;
  if (fd_ >= 0) {
    const int flags = ::fcntl(fd_, F_GETFL);
    if (flags >= 0) {
      ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    }
  }
}

void HardwareSerial::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool HardwareSerial::connected() const { return fd_ >= 0; }

int HardwareSerial::nativeFd() const { return fd_; }

int HardwareSerial::available() {
  if (fd_ < 0) {
    return 0;
  }
  if (baud_ > 0UL) {
    pumpRx();
    return static_cast<int>(rx_count_);
  }
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int rc = ::poll(&pfd, 1, 0);
  if (rc > 0 && (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0 &&
      (pfd.revents & POLLIN) == 0) {
    close();
    return 0;
  }
  if (rc <= 0 || (pfd.revents & POLLIN) == 0) {
    return 0;
  }
  int nbytes = 0;
  if (::ioctl(fd_, FIONREAD, &nbytes) == 0 && nbytes > 0) {
    return nbytes;
  }
  // Darwin PTY masters often report FIONREAD=0 while poll says POLLIN.
  return 1;
}

int HardwareSerial::read() {
  if (fd_ < 0) {
    return -1;
  }
  if (baud_ > 0UL) {
    pumpRx();
    return fifoPop();
  }
  uint8_t b = 0;
  const ssize_t n = ::read(fd_, &b, 1);
  if (n == 1) {
    return static_cast<int>(b);
  }
  if (n == 0) {
    close();
  }
  return -1;
}

size_t HardwareSerial::write(uint8_t b) {
  if (fd_ < 0) {
    return 0U;
  }
  waitTxSlot();
  for (;;) {
    const ssize_t n = ::write(fd_, &b, 1);
    if (n == 1) {
      return 1U;
    }
    if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd pfd{};
      pfd.fd = fd_;
      pfd.events = POLLOUT;
      if (::poll(&pfd, 1, 100) <= 0) {
        return 0U;
      }
      continue;
    }
    close();
    return 0U;
  }
}

HardwareSerial Serial;
