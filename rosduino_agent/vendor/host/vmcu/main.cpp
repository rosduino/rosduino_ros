// SPDX-License-Identifier: Apache-2.0
#include <Arduino.h>

#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <string>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <util.h>
#else
#include <pty.h>
#endif

void setup();
void loop();

namespace {

volatile std::sig_atomic_t g_running = 1;

void onSignal(int) { g_running = 0; }

void usage(const char* argv0) {
  std::fprintf(stderr,
               "Usage: %s --listen tcp://0.0.0.0:5760 | --listen pty\n"
               "Virtual MCU: runs the compiled Arduino sketch over a byte pipe.\n",
               argv0);
}

bool parseTcpListen(const std::string& rest, std::string* host, uint16_t* port) {
  const auto colon = rest.rfind(':');
  if (colon == std::string::npos || colon == 0U || colon + 1U >= rest.size()) {
    return false;
  }
  *host = rest.substr(0, colon);
  const long value = std::strtol(rest.c_str() + colon + 1U, nullptr, 10);
  if (value < 0 || value > 65535L) {
    return false;
  }
  *port = static_cast<uint16_t>(value);
  return !host->empty();
}

int listenTcp(const std::string& host, uint16_t port, uint16_t* bound_port) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    std::perror("socket");
    return -1;
  }
  const int one = 1;
  (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (host == "0.0.0.0" || host == "*") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    std::fprintf(stderr, "vMCU: invalid listen address %s\n", host.c_str());
    ::close(fd);
    return -1;
  }
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::perror("bind");
    ::close(fd);
    return -1;
  }
  if (::listen(fd, 1) != 0) {
    std::perror("listen");
    ::close(fd);
    return -1;
  }
  sockaddr_in got{};
  socklen_t len = sizeof(got);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&got), &len) == 0) {
    *bound_port = ntohs(got.sin_port);
  } else {
    *bound_port = port;
  }
  return fd;
}

int acceptClient(int listen_fd) {
  const int cfd = ::accept(listen_fd, nullptr, nullptr);
  if (cfd < 0) {
    if (errno == EINTR) {
      return -1;
    }
    std::perror("accept");
    return -1;
  }
  const int one = 1;
#if defined(SO_NOSIGPIPE)
  (void)::setsockopt(cfd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
  (void)::setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  return cfd;
}

bool openPty(std::string* slave_path) {
  int master = -1;
  int slave = -1;
  char name[128] = {};
  termios tio{};
  cfmakeraw(&tio);
  tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (openpty(&master, &slave, name, &tio, nullptr) != 0) {
    std::perror("openpty");
    return false;
  }
  // Keep the slave fd open: a Darwin PTY master returns EIO once no slave
  // remains, which would drop HELLO before the agent opens the path.
  (void)slave;
  Serial.attach(master);
  *slave_path = name;
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::string listen = "tcp://0.0.0.0:5760";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      return 0;
    }
    if (arg == "--listen" && i + 1 < argc) {
      listen = argv[++i];
      continue;
    }
    std::fprintf(stderr, "vMCU: unknown argument %s\n", argv[i]);
    usage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
#if defined(SIGPIPE)
  std::signal(SIGPIPE, SIG_IGN);
#endif

  int listen_fd = -1;
  const bool use_pty = (listen == "pty" || listen == "pty://");
  if (use_pty) {
    std::string slave;
    if (!openPty(&slave)) {
      return 1;
    }
    std::printf("PTY=%s\n", slave.c_str());
    std::fflush(stdout);
  } else {
    constexpr char kTcp[] = "tcp://";
    if (listen.compare(0, sizeof(kTcp) - 1U, kTcp) != 0) {
      std::fprintf(stderr, "vMCU: listen URI must be tcp://host:port or pty\n");
      return 2;
    }
    std::string host;
    uint16_t port = 0U;
    if (!parseTcpListen(listen.substr(sizeof(kTcp) - 1U), &host, &port)) {
      std::fprintf(stderr, "vMCU: invalid listen URI %s\n", listen.c_str());
      return 2;
    }
    uint16_t bound = 0U;
    listen_fd = listenTcp(host, port, &bound);
    if (listen_fd < 0) {
      return 1;
    }
    std::printf("LISTENING tcp://%s:%u\n", host.c_str(), static_cast<unsigned>(bound));
    std::fflush(stdout);
    std::fprintf(stderr, "vMCU: waiting for TCP client\n");
    while (g_running && !Serial.connected()) {
      const int cfd = acceptClient(listen_fd);
      if (cfd >= 0) {
        Serial.attach(cfd);
        std::fprintf(stderr, "vMCU: client connected\n");
      }
    }
    if (!g_running) {
      if (listen_fd >= 0) {
        ::close(listen_fd);
      }
      return 0;
    }
  }

  setup();

  while (g_running) {
    if (!use_pty && listen_fd >= 0 && !Serial.connected()) {
      const int cfd = acceptClient(listen_fd);
      if (cfd >= 0) {
        Serial.attach(cfd);
        std::fprintf(stderr, "vMCU: client reconnected\n");
      }
    }
    loop();
    delay(1);
  }

  Serial.close();
  if (listen_fd >= 0) {
    ::close(listen_fd);
  }
  return 0;
}
