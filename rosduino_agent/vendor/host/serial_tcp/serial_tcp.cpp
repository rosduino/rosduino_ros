// SPDX-License-Identifier: Apache-2.0
#include "serial_tcp/serial_tcp.h"

#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>

#include "transport/posix_io.h"

namespace rosduino {
namespace host {
namespace {

void setErrno(std::string* err, const char* prefix) {
  *err = std::string(prefix) + ": " + std::strerror(errno);
}

void applyClientOptions(int fd) {
  posix::setCloexec(fd);
  posix::setNonblock(fd);
  const int one = 1;
#if defined(SO_NOSIGPIPE)
  (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
  (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
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

SerialTcpBridge::SerialTcpBridge(Config cfg)
    : cfg_(std::move(cfg)),
      serial_(cfg_.serial_path, cfg_.baud, cfg_.serial_dtr),
      listen_fd_(-1),
      client_fd_(-1),
      bound_port_(0U) {}

SerialTcpBridge::~SerialTcpBridge() {
  closeClient();
  closeListen();
  serial_.close();
}

std::string SerialTcpBridge::describe() const {
  return cfg_.serial_path + " <-> tcp://" + cfg_.listen_host + ":" +
         std::to_string(bound_port_ == 0U ? cfg_.listen_port : bound_port_);
}

void SerialTcpBridge::closeListen() { posix::closeFd(listen_fd_); }

void SerialTcpBridge::closeClient() { posix::closeFd(client_fd_); }

bool SerialTcpBridge::bindListen() {
  closeListen();
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    setErrno(&err_, "socket");
    return false;
  }
  posix::setCloexec(listen_fd_);
  posix::setNonblock(listen_fd_);
  const int one = 1;
  (void)::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(cfg_.listen_port);
  if (cfg_.listen_host.empty() || cfg_.listen_host == "0.0.0.0" || cfg_.listen_host == "*") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (::inet_pton(AF_INET, cfg_.listen_host.c_str(), &addr.sin_addr) != 1) {
    err_ = "invalid listen host";
    closeListen();
    return false;
  }
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    setErrno(&err_, "bind");
    closeListen();
    return false;
  }
  if (::listen(listen_fd_, 1) != 0) {
    setErrno(&err_, "listen");
    closeListen();
    return false;
  }
  sockaddr_in got{};
  socklen_t len = sizeof(got);
  if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&got), &len) == 0) {
    bound_port_ = ntohs(got.sin_port);
  } else {
    bound_port_ = cfg_.listen_port;
  }
  err_.clear();
  return true;
}

bool SerialTcpBridge::start() {
  if (cfg_.serial_path.empty()) {
    err_ = "empty serial path";
    return false;
  }
  (void)serial_.open();
  return bindListen();
}

void SerialTcpBridge::pulseDtr() {
  if (!serial_.isOpen()) {
    return;
  }
  const int fd = serial_.nativeFd();
  int bits = 0;
  if (::ioctl(fd, TIOCMGET, &bits) != 0) {
    return;
  }
  bits &= ~(TIOCM_DTR | TIOCM_RTS);
  (void)::ioctl(fd, TIOCMSET, &bits);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  bits |= TIOCM_DTR | TIOCM_RTS;
  (void)::ioctl(fd, TIOCMSET, &bits);
}

void SerialTcpBridge::acceptClient() {
  const int fd = ::accept(listen_fd_, nullptr, nullptr);
  if (fd < 0) {
    return;
  }
  if (client_fd_ >= 0) {
    ::close(fd);
    return;
  }
  applyClientOptions(fd);
  client_fd_ = fd;
  if (cfg_.reset_on_connect) {
    pulseDtr();
  }
}

void SerialTcpBridge::pumpSerialToClient() {
  uint8_t buf[512];
  const ssize_t n = serial_.read(buf, sizeof(buf));
  if (n < 0) {
    return;
  }
  if (n == 0 || client_fd_ < 0) {
    return;
  }
  if (sendAll(client_fd_, buf, static_cast<size_t>(n)) < 0) {
    closeClient();
  }
}

void SerialTcpBridge::pumpClientToSerial() {
  uint8_t buf[512];
  const ssize_t n = ::recv(client_fd_, buf, sizeof(buf), 0);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return;
    }
    closeClient();
    return;
  }
  if (n == 0) {
    closeClient();
    return;
  }
  if (!serial_.ensureOpen()) {
    return;
  }
  if (serial_.write(buf, static_cast<size_t>(n)) < 0) {
    (void)serial_.ensureOpen();
  }
}

void SerialTcpBridge::run(std::atomic<bool>& running) {
  while (running.load()) {
    (void)serial_.ensureOpen();

    pollfd pfds[3];
    nfds_t nfd = 0;
    const nfds_t listen_i = nfd;
    pfds[nfd].fd = listen_fd_;
    pfds[nfd].events = POLLIN;
    pfds[nfd].revents = 0;
    ++nfd;

    int serial_i = -1;
    if (serial_.isOpen()) {
      serial_i = static_cast<int>(nfd);
      pfds[nfd].fd = serial_.nativeFd();
      pfds[nfd].events = POLLIN;
      pfds[nfd].revents = 0;
      ++nfd;
    }

    int client_i = -1;
    if (client_fd_ >= 0) {
      client_i = static_cast<int>(nfd);
      pfds[nfd].fd = client_fd_;
      pfds[nfd].events = POLLIN;
      pfds[nfd].revents = 0;
      ++nfd;
    }

    const int rc = ::poll(pfds, nfd, 50);
    if (rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      setErrno(&err_, "poll");
      break;
    }

    if ((pfds[listen_i].revents & POLLIN) != 0) {
      acceptClient();
    }
    if (serial_i >= 0 && (pfds[serial_i].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
      pumpSerialToClient();
    }
    if (client_i >= 0 && (pfds[client_i].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
      pumpClientToSerial();
    }
  }
  closeClient();
}

}  // namespace host
}  // namespace rosduino
