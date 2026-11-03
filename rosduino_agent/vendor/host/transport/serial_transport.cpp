// SPDX-License-Identifier: Apache-2.0
#include "transport/serial_transport.h"

#include <cstring>
#include <sys/ioctl.h>
#include <termios.h>

#if defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#endif

#include "transport/posix_io.h"

namespace rosduino {
namespace host {
namespace {

bool baudToSpeed(uint32_t baud, speed_t* out) {
  switch (baud) {
    case 9600U:
      *out = B9600;
      return true;
    case 19200U:
      *out = B19200;
      return true;
    case 38400U:
      *out = B38400;
      return true;
    case 57600U:
      *out = B57600;
      return true;
    case 115200U:
      *out = B115200;
      return true;
    case 230400U:
      *out = B230400;
      return true;
#ifdef B250000
    case 250000U:
      *out = B250000;
      return true;
#endif
#ifdef B460800
    case 460800U:
      *out = B460800;
      return true;
#endif
#ifdef B500000
    case 500000U:
      *out = B500000;
      return true;
#endif
#ifdef B921600
    case 921600U:
      *out = B921600;
      return true;
#endif
    default:
      return false;
  }
}

void setErrno(std::string* err, const char* prefix) {
  *err = std::string(prefix) + ": " + std::strerror(errno);
}

}  // namespace

SerialTransport::SerialTransport(std::string path, uint32_t baud, bool dtr)
    : path_(std::move(path)), baud_(baud), dtr_(dtr), fd_(-1) {}

SerialTransport::~SerialTransport() { close(); }

bool SerialTransport::isOpen() const { return fd_ >= 0; }

int SerialTransport::nativeFd() const { return fd_; }

std::string SerialTransport::describe() const { return path_; }

const char* SerialTransport::lastError() const { return err_.c_str(); }

void SerialTransport::close() { posix::closeFd(fd_); }

void SerialTransport::drop() { posix::closeFd(fd_); }

bool SerialTransport::ensureOpen() {
  if (isOpen()) {
    return true;
  }
  return open();
}

bool SerialTransport::configureTermios() {
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    setErrno(&err_, "tcgetattr");
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
#ifdef CRTSCTS
  tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
#endif
  tio.c_cflag &= ~static_cast<tcflag_t>(CSIZE | PARENB | CSTOPB);
  tio.c_cflag |= CS8;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  speed_t speed = B115200;
  const bool known = baudToSpeed(baud_, &speed);
#if defined(__APPLE__)
  const bool apple_custom = (baud_ == 250000U || baud_ == 460800U ||
                             baud_ == 500000U || baud_ == 921600U);
#endif
  if (!known) {
#if defined(__APPLE__)
    if (!apple_custom) {
      err_ = "unsupported baud rate";
      return false;
    }
#else
    err_ = "unsupported baud rate";
    return false;
#endif
  }
  if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
    setErrno(&err_, "cfsetpeed");
    return false;
  }
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    setErrno(&err_, "tcsetattr");
    return false;
  }
#if defined(__APPLE__)
  if (apple_custom) {
    speed_t custom = static_cast<speed_t>(baud_);
    if (ioctl(fd_, IOSSIOSPEED, &custom) != 0) {
      setErrno(&err_, "IOSSIOSPEED");
      return false;
    }
  }
#endif
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialTransport::applyDtr() {
  if (!dtr_) {
    // false means "do not touch modem bits". Clearing DTR/RTS on a PTY
    // hangs up the master (EIO), which is the opposite of what vMCU needs.
    return;
  }
  int bits = 0;
  if (ioctl(fd_, TIOCMGET, &bits) != 0) {
    return;  // PTYs and some USB-CDC devices have no modem bits.
  }
  bits |= TIOCM_DTR | TIOCM_RTS;
  (void)ioctl(fd_, TIOCMSET, &bits);
}

bool SerialTransport::open() {
  close();
  if (path_.empty()) {
    err_ = "empty serial path";
    return false;
  }
  fd_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    setErrno(&err_, "open");
    return false;
  }
  posix::setCloexec(fd_);
  posix::setNonblock(fd_);
  if (!configureTermios()) {
    posix::closeFd(fd_);
    return false;
  }
  applyDtr();
  err_.clear();
  return true;
}

bool SerialTransport::waitReadable(int timeout_ms) {
  if (!isOpen()) {
    return false;
  }
  const int rev = posix::pollOne(fd_, POLLIN, timeout_ms);
  if (rev < 0) {
    setErrno(&err_, "poll");
    drop();
    return false;
  }
  if (rev == 0) {
    return false;
  }
  if ((rev & POLLIN) != 0) {
    return true;
  }
  if ((rev & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    drop();
  }
  return false;
}

ssize_t SerialTransport::read(uint8_t* buf, size_t len) {
  if (!isOpen() || buf == nullptr) {
    return -1;
  }
  if (len == 0U) {
    return 0;
  }
  const ssize_t n = ::read(fd_, buf, len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    setErrno(&err_, "read");
    drop();
    return -1;
  }
  if (n == 0) {
    err_ = "serial hangup";
    drop();
    return -1;
  }
  return n;
}

ssize_t SerialTransport::write(const uint8_t* buf, size_t len) {
  if (!isOpen() || (len > 0U && buf == nullptr)) {
    return -1;
  }
  if (len == 0U) {
    return 0;
  }
  const ssize_t n = posix::writeAll(fd_, buf, len);
  if (n < 0) {
    setErrno(&err_, "write");
    drop();
    return -1;
  }
  return n;
}

}  // namespace host
}  // namespace rosduino
