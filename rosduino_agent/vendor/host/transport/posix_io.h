// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_POSIX_IO_H_
#define ROSDUINO_HOST_POSIX_IO_H_

#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace rosduino {
namespace host {
namespace posix {

inline void closeFd(int& fd) {
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

inline void setCloexec(int fd) {
  const int flags = ::fcntl(fd, F_GETFD);
  if (flags >= 0) {
    ::fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
  }
}

inline void setNonblock(int fd) {
  const int flags = ::fcntl(fd, F_GETFL);
  if (flags >= 0) {
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
}

inline int pollOne(int fd, short events, int timeout_ms) {
  pollfd pfd{};
  pfd.fd = fd;
  pfd.events = events;
  int rc;
  do {
    rc = ::poll(&pfd, 1, timeout_ms);
  } while (rc < 0 && errno == EINTR);
  if (rc <= 0) {
    return rc;
  }
  return pfd.revents;
}

inline ssize_t writeAll(int fd, const uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::write(fd, buf + off, len - off);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        const int rev = pollOne(fd, POLLOUT, 1000);
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

}  // namespace posix
}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_POSIX_IO_H_
