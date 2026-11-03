// SPDX-License-Identifier: Apache-2.0
#include "transport/transport.h"

#include <cctype>
#include <cstdlib>

#include "transport/serial_transport.h"
#include "transport/tcp_transport.h"

namespace rosduino {
namespace host {
namespace {

bool parseTcpEndpoint(const std::string& rest, std::string* host, uint16_t* port) {
  if (rest.empty()) {
    return false;
  }
  std::string host_part;
  std::string port_part;
  if (rest.front() == '[') {
    const auto close = rest.find(']');
    if (close == std::string::npos || close + 1U >= rest.size() || rest[close + 1U] != ':') {
      return false;
    }
    host_part = rest.substr(1, close - 1U);
    port_part = rest.substr(close + 2U);
  } else {
    const auto colon = rest.rfind(':');
    if (colon == std::string::npos || colon == 0U || colon + 1U >= rest.size()) {
      return false;
    }
    host_part = rest.substr(0, colon);
    port_part = rest.substr(colon + 1U);
  }
  if (host_part.empty() || port_part.empty()) {
    return false;
  }
  for (char c : port_part) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return false;
    }
  }
  const long value = std::strtol(port_part.c_str(), nullptr, 10);
  if (value <= 0 || value > 65535L) {
    return false;
  }
  *host = host_part;
  *port = static_cast<uint16_t>(value);
  return true;
}

}  // namespace

std::unique_ptr<Transport> makeTransport(const TransportConfig& cfg) {
  constexpr char kTcpPrefix[] = "tcp://";
  if (cfg.endpoint.compare(0, sizeof(kTcpPrefix) - 1U, kTcpPrefix) == 0) {
    std::string host;
    uint16_t port = 0U;
    if (!parseTcpEndpoint(cfg.endpoint.substr(sizeof(kTcpPrefix) - 1U), &host, &port)) {
      return std::make_unique<TcpClientTransport>("", 0U, cfg.reconnect_interval);
    }
    return std::make_unique<TcpClientTransport>(host, port, cfg.reconnect_interval);
  }
  return std::make_unique<SerialTransport>(cfg.endpoint, cfg.baud, cfg.dtr);
}

}  // namespace host
}  // namespace rosduino
