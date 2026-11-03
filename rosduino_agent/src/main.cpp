// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "bridge/agent_session.h"
#include "diagnostics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_backend.hpp"
#include "rcutils/logging.h"
#include "transport/transport.h"

namespace {

using rosduino::agent::DiagnosticsPublisher;
using rosduino::agent::RclcppBackend;
using rosduino::host::AgentConfig;
using rosduino::host::AgentSession;
using rosduino::host::Transport;
using rosduino::host::TransportConfig;
using rosduino::host::makeTransport;

struct Cli {
  std::optional<std::string> port;
  std::optional<int64_t> baud;
  std::optional<std::string> ns;
  std::optional<bool> keep_on_disconnect;
  std::optional<int64_t> service_timeout_ms;
  std::optional<bool> diagnostics;
  std::optional<std::string> log_level;
  bool help = false;
};

struct TransportSink {
  Transport* transport = nullptr;
  std::mutex* mu = nullptr;

  bool writeByte(uint8_t b) {
    if (transport == nullptr) {
      return false;
    }
    if (mu != nullptr) {
      std::lock_guard<std::mutex> lock(*mu);
      return transport->write(&b, 1) == 1;
    }
    return transport->write(&b, 1) == 1;
  }
};

void usage(const char* argv0) {
  std::fprintf(stderr,
               "Usage: %s --port PATH|tcp://host:port [options]\n"
               "  --baud N                  serial baud (default 115200)\n"
               "  --namespace NS            MCU node namespace (overrides HELLO ns)\n"
               "  --keep-on-disconnect      keep ROS entities if the MCU drops\n"
               "  --service-timeout-ms N    service call timeout (default 5000)\n"
               "  --no-diagnostics          do not publish /diagnostics\n"
               "  --log-level LEVEL         debug|info|warn|error|fatal\n",
               argv0);
}

bool parseCli(int argc, char** argv, Cli* cli, std::vector<char*>* rcl_argv) {
  rcl_argv->push_back(argv[0]);
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto take = [&](std::optional<std::string>* out) -> bool {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "rosduino_agent: missing value for %s\n", arg.c_str());
        return false;
      }
      *out = argv[++i];
      return true;
    };
    if (arg == "--help" || arg == "-h") {
      cli->help = true;
      return true;
    }
    if (arg == "--port") {
      if (!take(&cli->port)) {
        return false;
      }
      continue;
    }
    if (arg == "--baud") {
      std::optional<std::string> raw;
      if (!take(&raw)) {
        return false;
      }
      cli->baud = std::strtoll(raw->c_str(), nullptr, 10);
      continue;
    }
    if (arg == "--namespace") {
      if (!take(&cli->ns)) {
        return false;
      }
      continue;
    }
    if (arg == "--keep-on-disconnect") {
      cli->keep_on_disconnect = true;
      continue;
    }
    if (arg == "--service-timeout-ms") {
      std::optional<std::string> raw;
      if (!take(&raw)) {
        return false;
      }
      cli->service_timeout_ms = std::strtoll(raw->c_str(), nullptr, 10);
      continue;
    }
    if (arg == "--no-diagnostics") {
      cli->diagnostics = false;
      continue;
    }
    if (arg == "--log-level") {
      if (!take(&cli->log_level)) {
        return false;
      }
      continue;
    }
    rcl_argv->push_back(argv[i]);
  }
  return true;
}

int logLevelFromName(const std::string& name) {
  if (name == "debug") {
    return RCUTILS_LOG_SEVERITY_DEBUG;
  }
  if (name == "info") {
    return RCUTILS_LOG_SEVERITY_INFO;
  }
  if (name == "warn" || name == "warning") {
    return RCUTILS_LOG_SEVERITY_WARN;
  }
  if (name == "error") {
    return RCUTILS_LOG_SEVERITY_ERROR;
  }
  if (name == "fatal") {
    return RCUTILS_LOG_SEVERITY_FATAL;
  }
  return RCUTILS_LOG_SEVERITY_INFO;
}

uint32_t nowMs() {
  using clock = std::chrono::steady_clock;
  static const clock::time_point t0 = clock::now();
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - t0).count());
}

void applyLogLevel(const std::string& name) {
  const rcutils_ret_t rc =
      rcutils_logging_set_logger_level("rosduino_agent", logLevelFromName(name));
  if (rc != RCUTILS_RET_OK) {
    std::fprintf(stderr, "rosduino_agent: failed to set log level '%s'\n", name.c_str());
  }
}

}  // namespace

int main(int argc, char** argv) {
  Cli cli;
  std::vector<char*> rcl_argv;
  if (!parseCli(argc, argv, &cli, &rcl_argv)) {
    usage(argv[0]);
    return 2;
  }
  if (cli.help) {
    usage(argv[0]);
    return 0;
  }

  int rcl_argc = static_cast<int>(rcl_argv.size());
  rclcpp::init(rcl_argc, rcl_argv.data());

  auto agent_node = std::make_shared<rclcpp::Node>("rosduino_agent");
  agent_node->declare_parameter("port", std::string(""));
  agent_node->declare_parameter("baud", static_cast<int64_t>(115200));
  agent_node->declare_parameter("namespace", std::string(""));
  agent_node->declare_parameter("keep_on_disconnect", false);
  agent_node->declare_parameter("service_timeout_ms", static_cast<int64_t>(5000));
  agent_node->declare_parameter("diagnostics", true);
  agent_node->declare_parameter("log_level", std::string("info"));

  auto set_if = [&](const char* name, const auto& opt) {
    if (opt.has_value()) {
      agent_node->set_parameter(rclcpp::Parameter(name, *opt));
    }
  };
  set_if("port", cli.port);
  set_if("baud", cli.baud);
  set_if("namespace", cli.ns);
  set_if("keep_on_disconnect", cli.keep_on_disconnect);
  set_if("service_timeout_ms", cli.service_timeout_ms);
  set_if("diagnostics", cli.diagnostics);
  set_if("log_level", cli.log_level);

  const std::string port = agent_node->get_parameter("port").as_string();
  const uint32_t baud =
      static_cast<uint32_t>(agent_node->get_parameter("baud").as_int());
  const std::string ns = agent_node->get_parameter("namespace").as_string();
  const bool keep = agent_node->get_parameter("keep_on_disconnect").as_bool();
  const bool diagnostics = agent_node->get_parameter("diagnostics").as_bool();
  const std::string log_level = agent_node->get_parameter("log_level").as_string();
  const int64_t timeout_param = agent_node->get_parameter("service_timeout_ms").as_int();
  const uint32_t service_timeout_ms =
      (timeout_param < 1) ? 5000U : static_cast<uint32_t>(timeout_param);

  applyLogLevel(log_level);

  if (port.empty()) {
    RCLCPP_ERROR(agent_node->get_logger(), "parameter 'port' is required");
    usage(argv[0]);
    rclcpp::shutdown();
    return 2;
  }
  if (!diagnostics) {
    RCLCPP_INFO(agent_node->get_logger(), "/diagnostics disabled");
  }

  TransportConfig tcfg;
  tcfg.endpoint = port;
  tcfg.baud = baud;
  auto transport = makeTransport(tcfg);
  if (transport == nullptr) {
    RCLCPP_ERROR(agent_node->get_logger(), "unsupported port '%s'", port.c_str());
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(agent_node);
  RclcppBackend backend(agent_node, executor, ns, service_timeout_ms);
  AgentConfig acfg;
  acfg.keep_on_disconnect = keep;
  acfg.service_timeout_ms = service_timeout_ms;
  AgentSession session(backend, acfg);
  session.setTimeSource([&](int32_t* sec, uint32_t* nsec) {
    const int64_t ns_count = agent_node->now().nanoseconds();
    *sec = static_cast<int32_t>(ns_count / 1000000000LL);
    *nsec = static_cast<uint32_t>(ns_count % 1000000000LL);
  });

  std::unique_ptr<DiagnosticsPublisher> diagnostics_pub;
  if (diagnostics) {
    diagnostics_pub =
        std::make_unique<DiagnosticsPublisher>(agent_node, session, *transport, port);
    RCLCPP_INFO(agent_node->get_logger(), "publishing /diagnostics at 1 Hz");
  }

  std::mutex write_mu;
  std::atomic<bool> running{true};
  std::thread io([&]() {
    TransportSink sink{transport.get(), &write_mu};
    while (running.load() && rclcpp::ok()) {
      if (!transport->ensureOpen()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }
      session.tick(nowMs(), sink);
      if (transport->waitReadable(20)) {
        uint8_t buf[256];
        const ssize_t n = transport->read(buf, sizeof(buf));
        if (n < 0) {
          continue;
        }
        for (ssize_t i = 0; i < n; ++i) {
          session.pushRxByte(buf[i], nowMs(), sink);
        }
      }
    }
  });

  executor.spin();
  running = false;
  if (io.joinable()) {
    io.join();
  }

  if (transport->isOpen()) {
    TransportSink sink{transport.get(), &write_mu};
    session.shutdown(sink);
  }
  transport->close();
  rclcpp::shutdown();
  return 0;
}
