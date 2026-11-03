// SPDX-License-Identifier: Apache-2.0
#include "transport/tcp_transport.h"

#include <arpa/inet.h>
#include <cstring>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "transport/posix_io.h"

namespace rosduino {
namespace host {
namespace {

constexpr int kConnectTimeoutMs = 2000;

void setErrno(std::string* err, const char* prefix) {
  *err = std::string(prefix) + ": " + std::strerror(errno);
}

void applySocketOptions(int fd) {
  posix::setCloexec(fd);
  posix::setNonblock(fd);
  const int one = 1;
#if defined(SO_NOSIGPIPE)
  (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
  (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
}

bool connectWithTimeout(int fd, const sockaddr* addr, socklen_t addrlen, std::string* err) {
  const int rc = ::connect(fd, addr, addrlen);
  if (rc == 0) {
    return true;
  }
  if (errno != EINPROGRESS && errno != EINTR) {
    setErrno(err, "connect");
    return false;
  }
  const int rev = posix::pollOne(fd, POLLOUT, kConnectTimeoutMs);
  if (rev <= 0) {
    *err = (rev == 0) ? "connect timeout" : (std::string("connect poll: ") + std::strerror(errno));
    return false;
  }
  if ((rev & (POLLERR | POLLHUP | POLLNVAL)) != 0 && (rev & POLLOUT) == 0) {
    *err = "connect failed";
    return false;
  }
  int soerr = 0;
  socklen_t slen = sizeof(soerr);
  if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &slen) != 0) {
    setErrno(err, "getsockopt");
    return false;
  }
  if (soerr != 0) {
    errno = soerr;
    setErrno(err, "connect");
    return false;
  }
  return true;
}

ssize_t sendAll(int fd, const uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
#if defined(MSG_NOSIGNAL)
    const int flags = MSG_NOSIGNAL;
#else
    const int flags = 0;
#endif
    const ssize_t n = ::send(fd, buf + off, len - off, flags);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        const int rev = posix::pollOne(fd, POLLOUT, 1000);
        if (rev <= 0 || (rev & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
          return -1;
        }
        continue;
      }
      return -1;
    }
    if (n == 0) {
      return -1;
    }
    off += static_cast<size_t>(n);
  }
  return static_cast<ssize_t>(off);
}

}  // namespace

TcpClientTransport::TcpClientTransport(std::string host, uint16_t port,
                                       std::chrono::milliseconds reconnect_interval)
    : host_(std::move(host)),
      port_(port),
      reconnect_interval_(reconnect_interval),
      throttle_reconnect_(false),
      fd_(-1) {}

TcpClientTransport::~TcpClientTransport() { close(); }

bool TcpClientTransport::isOpen() const { return fd_ >= 0; }

int TcpClientTransport::nativeFd() const { return fd_; }

std::string TcpClientTransport::describe() const {
  return "tcp://" + host_ + ":" + std::to_string(port_);
}

const char* TcpClientTransport::lastError() const { return err_.c_str(); }

void TcpClientTransport::close() {
  posix::closeFd(fd_);
  throttle_reconnect_ = false;
}

void TcpClientTransport::drop() {
  posix::closeFd(fd_);
  last_attempt_ = std::chrono::steady_clock::now();
  throttle_reconnect_ = true;
}

bool TcpClientTransport::ensureOpen() {
  if (isOpen()) {
    return true;
  }
  if (throttle_reconnect_) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_attempt_ < reconnect_interval_) {
      return false;
    }
  }
  return open();
}

bool TcpClientTransport::open() {
  close();
  last_attempt_ = std::chrono::steady_clock::now();
  if (host_.empty() || port_ == 0U) {
    err_ = "invalid tcp endpoint";
    throttle_reconnect_ = true;
    return false;
  }

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  addrinfo* res = nullptr;
  const std::string port_str = std::to_string(port_);
  const int gerr = ::getaddrinfo(host_.c_str(), port_str.c_str(), &hints, &res);
  if (gerr != 0) {
    err_ = std::string("getaddrinfo: ") + gai_strerror(gerr);
    throttle_reconnect_ = true;
    return false;
  }

  int fd = -1;
  for (addrinfo* ai = res; ai != nullptr; ai = ai->ai_next) {
    fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (fd < 0) {
      setErrno(&err_, "socket");
      continue;
    }
    applySocketOptions(fd);
    if (connectWithTimeout(fd, ai->ai_addr, ai->ai_addrlen, &err_)) {
      break;
    }
    posix::closeFd(fd);
  }
  ::freeaddrinfo(res);

  if (fd < 0) {
    throttle_reconnect_ = true;
    return false;
  }
  fd_ = fd;
  throttle_reconnect_ = false;
  err_.clear();
  return true;
}

bool TcpClientTransport::waitReadable(int timeout_ms) {
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

ssize_t TcpClientTransport::read(uint8_t* buf, size_t len) {
  if (!isOpen() || buf == nullptr) {
    return -1;
  }
  if (len == 0U) {
    return 0;
  }
  const ssize_t n = ::recv(fd_, buf, len, 0);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    setErrno(&err_, "recv");
    drop();
    return -1;
  }
  if (n == 0) {
    err_ = "tcp hangup";
    drop();
    return -1;
  }
  return n;
}

ssize_t TcpClientTransport::write(const uint8_t* buf, size_t len) {
  if (!isOpen() || (len > 0U && buf == nullptr)) {
    return -1;
  }
  if (len == 0U) {
    return 0;
  }
  const ssize_t n = sendAll(fd_, buf, len);
  if (n < 0) {
    setErrno(&err_, "send");
    drop();
    return -1;
  }
  return n;
}

}  // namespace host
}  // namespace rosduino
