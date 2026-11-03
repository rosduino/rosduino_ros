// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_HOST_AGENT_SESSION_H_
#define ROSDUINO_HOST_AGENT_SESSION_H_

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "bridge/ros_backend.h"
#include "bridge/write_queue.h"
#include "rosduino/core/commands.h"
#include "rosduino/core/frame.h"

namespace rosduino {
namespace host {

enum class AgentState : uint8_t {
  WaitingHello = 0U,
  Active,
};

struct AgentConfig {
  uint16_t rx_max = 2048U;
  uint16_t sync_interval_ms = 500U;
  uint16_t rx_timeout_ms = 5000U;
  uint32_t service_timeout_ms = 5000U;
  bool keep_on_disconnect = false;
};

struct AgentSnapshot {
  AgentState state = AgentState::WaitingHello;
  bool active = false;
  std::string node_name;
  uint8_t proto_ver = 0U;
  uint8_t flags = 0U;
  uint8_t mcu_profile = 0U;
  uint8_t max_entities = 0U;
  uint8_t entity_count = 0U;
  uint8_t publishers = 0U;
  uint8_t subscribers = 0U;
  uint8_t service_servers = 0U;
  uint8_t service_clients = 0U;
  uint8_t entity_errors = 0U;
  uint32_t rx_frames = 0U;
  uint32_t tx_frames = 0U;
  uint32_t crc_errors = 0U;
  uint32_t cobs_errors = 0U;
  uint32_t losses = 0U;
  uint32_t no_resources = 0U;
  uint32_t too_large = 0U;
  uint32_t last_pong_age_ms = 0xFFFFFFFFU;
};

/**
 * @brief Agent-side session (`docs/protocol.md` section 5.2).
 *
 * Outgoing frames are enqueued and flushed by the caller (the single writer).
 */
class AgentSession {
 public:
  using TimeSource = std::function<void(int32_t* sec, uint32_t* nsec)>;

  explicit AgentSession(RosBackend& backend, AgentConfig cfg = AgentConfig());
  ~AgentSession();

  AgentSession(const AgentSession&) = delete;
  AgentSession& operator=(const AgentSession&) = delete;

  void setTimeSource(TimeSource fn) { time_source_ = std::move(fn); }

  AgentState state() const { return state_; }
  bool active() const { return state_ == AgentState::Active; }
  std::optional<uint16_t> sessionId() const { return session_; }
  uint8_t maxEntities() const { return max_entities_; }
  AgentSnapshot snapshot() const;

  template <typename Sink>
  core::FrameStatus pushRxByte(uint8_t byte, uint32_t now_ms, Sink& sink) {
    core::FrameView view{};
    const core::FrameStatus st = reader_.pushByte(byte, now_ms, &view);
    if (st == core::FrameStatus::FrameReady) {
      last_rx_ms_ = now_ms;
      handleFrame(view, now_ms);
      backend_.pollServiceClients();
      drainIncoming();
      drainServiceToMcu(now_ms);
      drainServiceFromRos();
      expireServiceRequests(now_ms);
      tx_frames_ += queue_.flush(writer_, peer_rx_max_, sink);
    }
    last_now_ms_ = now_ms;
    refreshCache();
    return st;
  }

  template <typename Sink>
  void shutdown(Sink& sink) {
    uint8_t body[1] = {static_cast<uint8_t>(core::ResetReason::AgentShutdown)};
    queue_.push(static_cast<uint8_t>(core::CommandType::ResetSession), nextSeq(), body, 1U);
    const size_t peer = (peer_rx_max_ == 0U) ? cfg_.rx_max : peer_rx_max_;
    tx_frames_ += queue_.flush(writer_, peer, sink);
    dropSession();
    refreshCache();
  }

  template <typename Sink>
  void tick(uint32_t now_ms, Sink& sink) {
    if (state_ == AgentState::WaitingHello) {
      if (!sync_sent_ ||
          static_cast<uint32_t>(now_ms - last_sync_ms_) >= cfg_.sync_interval_ms) {
        enqueueEmpty(core::CommandType::SyncRequest);
        last_sync_ms_ = now_ms;
        sync_sent_ = true;
      }
    } else if (state_ == AgentState::Active) {
      if (static_cast<uint32_t>(now_ms - last_rx_ms_) >= cfg_.rx_timeout_ms) {
        onRxTimeout();
        enqueueEmpty(core::CommandType::SyncRequest);
        last_sync_ms_ = now_ms;
        sync_sent_ = true;
      }
    }
    backend_.pollServiceClients();
    drainIncoming();
    drainServiceToMcu(now_ms);
    drainServiceFromRos();
    expireServiceRequests(now_ms);
    tx_frames_ += queue_.flush(writer_, peer_rx_max_, sink);
    last_now_ms_ = now_ms;
    refreshCache();
  }

 private:
  enum class EntityKind : uint8_t {
    None = 0U,
    Publisher,
    Subscriber,
    ServiceServer,
    ServiceClient,
  };

  struct Entity {
    EntityKind kind = EntityKind::None;
    bool occupied = false;
    std::string name;
    std::string type;
    uint64_t hash = 0ULL;
    uint8_t qos = 0U;
    uint8_t depth = 0U;
    core::StatusCode last_status = core::StatusCode::Ok;
  };

  struct IncomingData {
    uint8_t id = 0U;
    std::vector<uint8_t> cdr;
  };

  struct IncomingService {
    uint8_t id = 0U;
    uint8_t req_id = 0U;
    std::vector<uint8_t> cdr;
  };

  struct PendingMcuService {
    uint8_t id = 0U;
    uint8_t req_id = 0U;
    uint32_t deadline_ms = 0U;
  };

  void handleFrame(const core::FrameView& view, uint32_t now_ms);
  void handleHello(const core::HelloCmd& cmd, uint8_t seq);
  void handleCreateTopic(const core::CreateTopicCmd& cmd, uint8_t seq, EntityKind kind);
  void handleCreateService(const core::CreateServiceCmd& cmd, uint8_t seq, EntityKind kind);
  void handleDestroy(uint8_t id, uint8_t seq);
  void handleData(const core::DataCmd& cmd);
  void handleServiceRequest(const core::ServiceDataCmd& cmd);
  void handleServiceResponse(const core::ServiceDataCmd& cmd);
  void drainIncoming();
  void drainServiceToMcu(uint32_t now_ms);
  void drainServiceFromRos();
  void expireServiceRequests(uint32_t now_ms);
  void onRxTimeout();
  void dropSession();
  void enqueueEmpty(core::CommandType type);
  void enqueueWelcome(uint16_t session, core::StatusCode status, uint8_t seq);
  void enqueueStatus(uint8_t ref_seq, uint8_t id, core::StatusCode code);
  void recordStatus(uint8_t id, core::StatusCode code);
  void refreshCache();
  uint8_t nextSeq();
  void resizeEntities(uint8_t max_entities);

  RosBackend& backend_;
  AgentConfig cfg_;
  TimeSource time_source_;
  std::vector<uint8_t> rx_buf_;
  std::vector<uint8_t> tx_buf_;
  core::FrameReader reader_;
  core::FrameWriter writer_;
  WriteQueue queue_;
  AgentState state_;
  std::optional<uint16_t> session_;
  uint8_t max_entities_;
  uint8_t flags_;
  size_t peer_rx_max_;
  uint8_t tx_seq_;
  uint32_t last_rx_ms_;
  uint32_t last_sync_ms_;
  uint32_t last_now_ms_;
  uint32_t last_pong_ms_;
  uint32_t tx_frames_;
  uint32_t no_resources_;
  uint32_t too_large_;
  uint8_t proto_ver_;
  std::string node_name_;
  bool sync_sent_;
  std::vector<Entity> entities_;
  mutable std::mutex snap_mu_;
  mutable AgentSnapshot cache_;
  mutable uint32_t last_pong_ms_cache_;
  mutable uint32_t last_now_ms_cache_;
  std::mutex incoming_mu_;
  std::vector<IncomingData> incoming_;
  std::mutex service_to_mcu_mu_;
  std::vector<IncomingService> service_to_mcu_;
  std::mutex service_from_ros_mu_;
  std::vector<IncomingService> service_from_ros_;
  std::vector<PendingMcuService> pending_mcu_services_;
};

}  // namespace host
}  // namespace rosduino

#endif  // ROSDUINO_HOST_AGENT_SESSION_H_
