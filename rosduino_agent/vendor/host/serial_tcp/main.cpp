// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "serial_tcp/serial_tcp.h"

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

void usage(const char* argv0) {
  std::fprintf(stderr,
               "Usage: %s --serial PATH [--baud 115200] [--listen 127.0.0.1:5760] "
               "[--reset-on-connect]\n"
               "Transparent serial ↔ TCP bridge. One TCP client; the serial port stays open.\n",
               argv0);
}

bool parseListen(const std::string& spec, std::string* host, uint16_t* port) {
  const auto colon = spec.rfind(':');
  if (colon == std::string::npos || colon == 0U || colon + 1U >= spec.size()) {
    return false;
  }
  *host = spec.substr(0, colon);
  const long value = std::strtol(spec.c_str() + colon + 1U, nullptr, 10);
  if (value < 0 || value > 65535L) {
    return false;
  }
  *port = static_cast<uint16_t>(value);
  return !host->empty();
}

}  // namespace

int main(int argc, char** argv) {
  rosduino::host::SerialTcpBridge::Config cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto take = [&](std::string* out) -> bool {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "rosduino-serial-tcp: missing value for %s\n", arg.c_str());
        return false;
      }
      *out = argv[++i];
      return true;
    };
    if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      return 0;
    }
    if (arg == "--serial") {
      if (!take(&cfg.serial_path)) {
        return 2;
      }
      continue;
    }
    if (arg == "--baud") {
      std::string raw;
      if (!take(&raw)) {
        return 2;
      }
      const long value = std::strtol(raw.c_str(), nullptr, 10);
      if (value <= 0 || value > 4000000L) {
        std::fprintf(stderr, "rosduino-serial-tcp: invalid baud %s\n", raw.c_str());
        return 2;
      }
      cfg.baud = static_cast<uint32_t>(value);
      continue;
    }
    if (arg == "--listen") {
      std::string raw;
      if (!take(&raw)) {
        return 2;
      }
      if (!parseListen(raw, &cfg.listen_host, &cfg.listen_port)) {
        std::fprintf(stderr, "rosduino-serial-tcp: invalid listen %s\n", raw.c_str());
        return 2;
      }
      continue;
    }
    if (arg == "--reset-on-connect") {
      cfg.reset_on_connect = true;
      continue;
    }
    std::fprintf(stderr, "rosduino-serial-tcp: unknown argument %s\n", arg.c_str());
    usage(argv[0]);
    return 2;
  }
  if (cfg.serial_path.empty()) {
    usage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
#if defined(SIGPIPE)
  std::signal(SIGPIPE, SIG_IGN);
#endif

  rosduino::host::SerialTcpBridge bridge(cfg);
  if (!bridge.start()) {
    std::fprintf(stderr, "rosduino-serial-tcp: %s\n", bridge.lastError());
    return 1;
  }
  std::printf("LISTENING tcp://%s:%u\n", cfg.listen_host.c_str(),
              static_cast<unsigned>(bridge.boundPort()));
  std::fflush(stdout);

  std::atomic<bool> running{true};
  std::thread waiter([&]() {
    while (g_stop == 0 && running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    running = false;
  });
  bridge.run(running);
  g_stop = 1;
  if (waiter.joinable()) {
    waiter.join();
  }
  return 0;
}
