// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_CORE_SESSION_H_
#define ROSDUINO_CORE_SESSION_H_

#include <stddef.h>
#include <stdint.h>

#include "rosduino/core/commands.h"
#include "rosduino/core/frame.h"

namespace rosduino {
namespace core {

/**
 * @brief MCU session states (`docs/protocol.md`, section 5.1).
 */
enum class SessionState : uint8_t {
  Disconnected = 0U,
  Registering,
  Connected,
};

/**
 * @brief Kind of entity registered on the MCU.
 */
enum class EntityKind : uint8_t {
  None = 0U,
  Publisher,
  Subscriber,
  ServiceServer,
  ServiceClient,
};

/**
 * @brief Registration status of a single MCU entity.
 */
enum class EntityRegState : uint8_t {
  Unused = 0U,
  Pending,
  WaitingStatus,
  Active,
  Error,
};

/**
 * @brief Descriptor for an entity managed by `McuSession`.
 *
 * Uses function pointers (`on_data`, `on_service`) instead of `virtual` methods to keep
 * RAM footprint minimal on AVR.
 * On AVR, `hash_ptr` points to 8 bytes of LE hash in PROGMEM (2 bytes instead of 8 bytes in RAM).
 */
struct EntitySlot {
  EntityKind kind;
  EntityRegState reg_state;
  StatusCode status_code;
  uint8_t qos;
  uint8_t depth;
  StrView name;
  StrView type;
  const uint8_t* hash_ptr;
  bool hash_in_progmem;
#if !defined(__AVR__)
  uint64_t hash;
#endif
  void* user_ctx;
  void (*on_data)(void* ctx, const uint8_t* cdr, size_t cdr_len);
  void (*on_service)(void* ctx, uint8_t req_id, const uint8_t* cdr, size_t cdr_len);

  EntitySlot() = default;

  StatusCode status() const { return status_code; }
};

/**
 * @brief Timing configuration for `McuSession` (`docs/protocol.md`, section 7).
 */
struct SessionTimings {
  uint16_t hello_interval_ms;
  uint16_t create_timeout_ms;
  uint8_t create_max_retries;
  uint16_t ping_interval_ms;
  uint16_t rx_timeout_ms;
  uint16_t time_sync_interval_ms;

  constexpr SessionTimings()
      : hello_interval_ms(1000U),
        create_timeout_ms(500U),
        create_max_retries(3U),
        ping_interval_ms(1000U),
        rx_timeout_ms(3000U),
        time_sync_interval_ms(10000U) {}
};

/**
 * @brief Synchronized ROS time snapshot with half-RTT compensation.
 *
 * Uses 32-bit arithmetic (without 64-bit software division `__udivmod64` on AVR).
 */
struct SyncedTime {
  int32_t base_sec;
  uint32_t base_nanosec;
  uint32_t sync_ms;
  bool valid;

  SyncedTime() = default;

  void getNow(uint32_t now_ms, int32_t* out_sec, uint32_t* out_nanosec) const {
    if (!valid) {
      *out_sec = 0;
      *out_nanosec = 0U;
      return;
    }
    const uint32_t elapsed_ms = static_cast<uint32_t>(now_ms - sync_ms);
    const uint32_t elapsed_sec = elapsed_ms / 1000U;
    const uint32_t rem_ms = elapsed_ms - (elapsed_sec * 1000U);
    uint32_t nsec = base_nanosec + (rem_ms * 1000000UL);
    int32_t sec = base_sec + static_cast<int32_t>(elapsed_sec);
    if (nsec >= 1000000000UL) {
      nsec -= 1000000000UL;
      ++sec;
    }
    *out_sec = sec;
    *out_nanosec = nsec;
  }
};

/**
 * @brief MCU protocol session state machine (`docs/protocol.md`, section 5.1).
 *
 * Independent of `Arduino.h`: time (`now_ms`) and byte output (`Sink`) are passed as arguments
 * so the state machine can be unit-tested deterministically on the host.
 */
class McuSession {
 public:
  using ConnectCallback = void (*)(void* ctx);

  static constexpr uint16_t kDefaultHelloIntervalMs = 1000U;
  static constexpr uint16_t kDefaultCreateTimeoutMs = 500U;
  static constexpr uint8_t kDefaultCreateMaxRetries = 3U;
  static constexpr uint16_t kDefaultPingIntervalMs = 1000U;
  static constexpr uint16_t kDefaultRxTimeoutMs = 3000U;
  static constexpr uint16_t kDefaultTimeSyncIntervalMs = 10000U;

  McuSession() = default;

  McuSession(uint8_t* rx_buf, size_t rx_max, uint8_t* tx_buf, size_t tx_max,
             EntitySlot* entities, uint8_t max_entities,
             const SessionTimings& /*timings*/ = SessionTimings())
      : reader_(rx_buf, rx_max),
        writer_(tx_buf, tx_max),
        entities_(entities),
        on_connect_(nullptr),
        on_disconnect_(nullptr),
        cb_ctx_(nullptr),
        last_tx_ms_(0U),
        last_rx_ms_(0U),
        timer_ms_(0U),
        synced_time_(),
        session_id_(0U),
        agent_rx_max_(0U),
        node_name_(),
        ns_(),
        max_entities_(max_entities),
        state_(SessionState::Disconnected),
        flags_(0U),
        tx_seq_(0U),
        reg_idx_(0U),
        reg_retries_(0U),
        reg_sent_seq_(0U),
        hello_sent_once_(false),
        time_req_pending_(false),
        notified_connected_(false) {}

  void attachBuffers(uint8_t* rx_buf, size_t rx_max, uint8_t* tx_buf, size_t tx_max,
                     EntitySlot* entities, uint8_t max_entities) {
    reader_.init(rx_buf, rx_max);
    writer_.init(tx_buf, tx_max);
    entities_ = entities;
    max_entities_ = max_entities;
  }

  void begin(uint16_t session_id, StrView node_name, StrView ns, uint8_t flags, uint32_t now_ms) {
    session_id_ = (session_id == 0U) ? 1U : session_id;
    node_name_ = node_name;
    ns_ = ns;
    flags_ = flags;
    state_ = SessionState::Disconnected;
    agent_rx_max_ = 0U;
    hello_sent_once_ = false;
    time_req_pending_ = false;
    notified_connected_ = false;
    last_rx_ms_ = now_ms;
    last_tx_ms_ = now_ms;
    timer_ms_ = now_ms;
    reader_.reset();
    resetEntitiesToPending();
  }

  void setCallbacks(ConnectCallback on_connect, ConnectCallback on_disconnect,
                    void* ctx = nullptr) {
    on_connect_ = on_connect;
    on_disconnect_ = on_disconnect;
    cb_ctx_ = ctx;
  }

  SessionState state() const { return state_; }

  bool connected() const {
    if (state_ != SessionState::Connected) {
      return false;
    }
    for (uint8_t i = 0U; i < max_entities_; ++i) {
      if (entities_[i].reg_state == EntityRegState::Pending ||
          entities_[i].reg_state == EntityRegState::WaitingStatus) {
        return false;
      }
    }
    return true;
  }

  StatusCode entityStatus(uint8_t id) const {
    if (entities_ == nullptr || id >= max_entities_) {
      return StatusCode::UnknownEntity;
    }
    return entities_[id].status();
  }

  uint16_t sessionId() const { return session_id_; }
  uint16_t agentRxMax() const { return agent_rx_max_; }
  const SyncedTime& syncedTime() const { return synced_time_; }
  FrameWriter& writer() { return writer_; }

  /**
   * @brief Allocate and register a new entity in the table.
   * @param out_id Populated with the slot index `0..max_entities_-1` on success.
   * @return `true` if a slot was available, `false` if `max_entities_` is exhausted.
   */
  bool registerEntity(const EntitySlot& spec, uint8_t* out_id) {
    for (uint8_t i = 0U; i < max_entities_; ++i) {
      if (entities_[i].reg_state == EntityRegState::Unused) {
        entities_[i] = spec;
        entities_[i].reg_state = EntityRegState::Pending;
        entities_[i].status_code = StatusCode::NotInSession;
        if (out_id != nullptr) {
          *out_id = i;
        }
        if (state_ == SessionState::Connected) {
          state_ = SessionState::Registering;
          reg_idx_ = i;
          reg_retries_ = 0U;
        }
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Destroy a registered entity and send `DESTROY_ENTITY` if in session.
   */
  template <typename Sink>
  bool destroyEntity(uint8_t id, uint32_t now_ms, Sink& sink) {
    if (id >= max_entities_ || entities_[id].reg_state == EntityRegState::Unused) {
      return false;
    }
    const bool was_registered = (entities_[id].reg_state == EntityRegState::Active);
    entities_[id] = EntitySlot();
    if (was_registered && state_ != SessionState::Disconnected) {
      DestroyEntityCmd cmd(id);
      uint8_t* body = writer_.beginBody();
      if (body != nullptr) {
        const size_t n = CommandCodec::encodeDestroyEntity(cmd, body, writer_.maxBodySize());
        if (n > 0U) {
          sendRawFrame(CommandType::DestroyEntity, nextSeq(), n, now_ms, sink);
        }
      }
    }
    return true;
  }

  /**
   * @brief Feed a single incoming wire byte and process the frame if complete.
   */
  template <typename Sink>
  FrameStatus pushRxByte(uint8_t byte, uint32_t now_ms, Sink& sink) {
    FrameView view;
    const FrameStatus st = reader_.pushByte(byte, now_ms, &view);
    if (st == FrameStatus::FrameReady) {
      handleFrame(view, now_ms, sink);
    }
    return st;
  }

  /**
   * @brief Advance timers (inter-byte timeout, RX loss timeout, HELLO retry, CREATE_* retry,
   *        PING keepalive, TIME_REQUEST).
   */
  template <typename Sink>
  void tick(uint32_t now_ms, Sink& sink) {
    reader_.checkTimeout(now_ms);

    if (state_ != SessionState::Disconnected) {
      if (static_cast<uint32_t>(now_ms - last_rx_ms_) >= kDefaultRxTimeoutMs) {
        transitionToDisconnected(now_ms);
      }
    }

    if (state_ == SessionState::Disconnected) {
      if (!hello_sent_once_ ||
          static_cast<uint32_t>(now_ms - timer_ms_) >= kDefaultHelloIntervalMs) {
        sendHello(now_ms, sink);
      }
      return;
    }

    if (state_ == SessionState::Registering) {
      stepRegistration(now_ms, sink);
      return;
    }

    if (state_ == SessionState::Connected) {
      if ((flags_ & kFlagNoTime) == 0U) {
        if (!time_req_pending_ &&
            static_cast<uint32_t>(now_ms - timer_ms_) >= kDefaultTimeSyncIntervalMs) {
          sendTimeRequest(now_ms, sink);
        }
      }

      if (static_cast<uint32_t>(now_ms - last_tx_ms_) >= kDefaultPingIntervalMs) {
        sendEmptyCommand(CommandType::Ping, nextSeq(), now_ms, sink);
      }
    }
  }

  /**
   * @brief Send a `LOG` frame if connected and `kFlagNoLog` is not set.
   */
  template <typename Sink>
  bool sendLog(LogLevel level, StrView text, uint32_t now_ms, Sink& sink) {
    if (state_ == SessionState::Disconnected || (flags_ & kFlagNoLog) != 0U) {
      return false;
    }
    uint8_t* body = writer_.beginBody();
    if (body == nullptr) {
      return false;
    }
    LogCmd cmd(level, text);
    const size_t len = CommandCodec::encodeLog(cmd, body, writer_.maxBodySize());
    if (len == 0U) {
      return false;
    }
    return sendRawFrame(CommandType::Log, nextSeq(), len, now_ms, sink);
  }

  /**
   * @brief Send a `DATA` frame. `tx_buf` must already contain `id` at byte 0 and `cdr_len`
   *        CDR bytes starting at byte 1 (written via `writer().beginBody()`).
   */
  template <typename Sink>
  bool sendData(uint8_t id, size_t cdr_len, uint32_t now_ms, Sink& sink) {
    if (state_ != SessionState::Connected) {
      return false;
    }
    if (id >= max_entities_ || entities_[id].kind != EntityKind::Publisher ||
        entities_[id].reg_state != EntityRegState::Active) {
      return false;
    }
    return sendRawFrame(CommandType::Data, nextSeq(), 1U + cdr_len, now_ms, sink);
  }

  /**
   * @brief Send a `SERVICE_RESPONSE` frame. `tx_buf` must already contain `id` and `req_id`
   *        at bytes 0–1 and `cdr_len` CDR bytes starting at byte 2.
   */
  template <typename Sink>
  bool sendServiceResponse(uint8_t id, size_t cdr_len, uint32_t now_ms, Sink& sink) {
    if (state_ != SessionState::Connected) {
      return false;
    }
    if (id >= max_entities_ || entities_[id].kind != EntityKind::ServiceServer ||
        entities_[id].reg_state != EntityRegState::Active) {
      return false;
    }
    return sendRawFrame(CommandType::ServiceResponse, nextSeq(), 2U + cdr_len, now_ms, sink);
  }

  /**
   * @brief Send a `SERVICE_REQUEST` frame. `tx_buf` must already contain `id` and `req_id`
   *        at bytes 0–1 and `cdr_len` CDR bytes starting at byte 2.
   */
  template <typename Sink>
  bool sendServiceRequest(uint8_t id, size_t cdr_len, uint32_t now_ms, Sink& sink) {
    if (state_ != SessionState::Connected) {
      return false;
    }
    if (id >= max_entities_ || entities_[id].kind != EntityKind::ServiceClient ||
        entities_[id].reg_state != EntityRegState::Active) {
      return false;
    }
    return sendRawFrame(CommandType::ServiceRequest, nextSeq(), 2U + cdr_len, now_ms, sink);
  }

 private:
  uint8_t nextSeq() {
    const uint8_t s = tx_seq_;
    tx_seq_ = static_cast<uint8_t>(tx_seq_ + 1U);
    return s;
  }

  void resetEntitiesToPending() {
    for (uint8_t i = 0U; i < max_entities_; ++i) {
      if (entities_[i].reg_state != EntityRegState::Unused) {
        entities_[i].reg_state = EntityRegState::Pending;
        entities_[i].status_code = StatusCode::NotInSession;
      }
    }
  }

  void transitionToDisconnected(uint32_t /*now_ms*/) {
    const bool was_connected = notified_connected_;
    state_ = SessionState::Disconnected;
    hello_sent_once_ = false;
    time_req_pending_ = false;
    notified_connected_ = false;
    resetEntitiesToPending();
    if (was_connected && on_disconnect_ != nullptr) {
      on_disconnect_(cb_ctx_);
    }
  }

  template <typename Sink>
  bool sendRawFrame(CommandType type, uint8_t seq, size_t body_len, uint32_t now_ms, Sink& sink) {
    if (!writer_.sendPreparedBody(static_cast<uint8_t>(type), seq, body_len, sink,
                                  agent_rx_max_)) {
      return false;
    }
    last_tx_ms_ = now_ms;
    return true;
  }

  template <typename Sink>
  bool sendEmptyCommand(CommandType type, uint8_t seq, uint32_t now_ms, Sink& sink) {
    if (!writer_.writeFrame(static_cast<uint8_t>(type), seq, nullptr, 0U, sink, agent_rx_max_)) {
      return false;
    }
    last_tx_ms_ = now_ms;
    return true;
  }

  template <typename Sink>
  void sendHello(uint32_t now_ms, Sink& sink) {
    uint8_t* body = writer_.beginBody();
    if (body == nullptr) {
      return;
    }
    HelloCmd cmd;
    cmd.proto_ver = kProtocolVersion;
    cmd.session = session_id_;
    cmd.rx_max = static_cast<uint16_t>(reader_.rxMax());
    cmd.tx_max = static_cast<uint16_t>(writer_.txMax());
    cmd.max_entities = max_entities_;
    cmd.flags = flags_;
    cmd.node_name = node_name_;
    cmd.ns = ns_;

    const size_t len = CommandCodec::encodeHello(cmd, body, writer_.maxBodySize());
    if (len > 0U) {
      if (writer_.sendPreparedBody(static_cast<uint8_t>(CommandType::Hello), nextSeq(), len, sink,
                                   0U)) {
        last_tx_ms_ = now_ms;
        timer_ms_ = now_ms;
        hello_sent_once_ = true;
      }
    }
  }

  template <typename Sink>
  void sendTimeRequest(uint32_t now_ms, Sink& sink) {
    if (sendEmptyCommand(CommandType::TimeRequest, nextSeq(), now_ms, sink)) {
      timer_ms_ = now_ms;
      time_req_pending_ = true;
    }
  }

  template <typename Sink>
  bool sendCreateEntity(uint8_t id, uint32_t now_ms, Sink& sink) {
    uint8_t* body = writer_.beginBody();
    if (body == nullptr) {
      return false;
    }
    const EntitySlot& slot = entities_[id];
    size_t len = 0U;
    CommandType cmd_type = CommandType::CreatePublisher;

    if (slot.kind == EntityKind::Publisher || slot.kind == EntityKind::Subscriber) {
      cmd_type = (slot.kind == EntityKind::Publisher) ? CommandType::CreatePublisher
                                                      : CommandType::CreateSubscriber;
      CreateTopicCmd cmd;
      cmd.id = id;
      cmd.qos = slot.qos;
      cmd.depth = slot.depth;
      cmd.name = slot.name;
      cmd.type = slot.type;
      cmd.hash_ptr = slot.hash_ptr;
      cmd.hash_in_progmem = slot.hash_in_progmem;
#if !defined(__AVR__)
      cmd.hash = slot.hash;
#endif
      len = CommandCodec::encodeCreateTopic(cmd, body, writer_.maxBodySize());
    } else if (slot.kind == EntityKind::ServiceServer ||
               slot.kind == EntityKind::ServiceClient) {
      cmd_type = (slot.kind == EntityKind::ServiceServer) ? CommandType::CreateServiceServer
                                                          : CommandType::CreateServiceClient;
      CreateServiceCmd cmd;
      cmd.id = id;
      cmd.name = slot.name;
      cmd.type = slot.type;
      cmd.hash_ptr = slot.hash_ptr;
      cmd.hash_in_progmem = slot.hash_in_progmem;
#if !defined(__AVR__)
      cmd.hash = slot.hash;
#endif
      len = CommandCodec::encodeCreateService(cmd, body, writer_.maxBodySize());
    } else {
      return false;
    }

    if (len == 0U) {
      return false;
    }
    const uint8_t seq = nextSeq();
    if (!sendRawFrame(cmd_type, seq, len, now_ms, sink)) {
      return false;
    }
    reg_sent_seq_ = seq;
    timer_ms_ = now_ms;
    return true;
  }

  template <typename Sink>
  void stepRegistration(uint32_t now_ms, Sink& sink) {
    while (reg_idx_ < max_entities_) {
      EntitySlot& slot = entities_[reg_idx_];
      if (slot.reg_state == EntityRegState::Unused ||
          slot.reg_state == EntityRegState::Active ||
          slot.reg_state == EntityRegState::Error) {
        ++reg_idx_;
        reg_retries_ = 0U;
        continue;
      }

      if (slot.reg_state == EntityRegState::Pending) {
        reg_retries_ = 0U;
        if (sendCreateEntity(reg_idx_, now_ms, sink)) {
          slot.reg_state = EntityRegState::WaitingStatus;
          return;
        }
        slot.reg_state = EntityRegState::Error;
        slot.status_code = StatusCode::TooLarge;
        ++reg_idx_;
        continue;
      }

      if (slot.reg_state == EntityRegState::WaitingStatus) {
        if (static_cast<uint32_t>(now_ms - timer_ms_) < kDefaultCreateTimeoutMs) {
          return;
        }
        if (reg_retries_ < kDefaultCreateMaxRetries) {
          ++reg_retries_;
          sendCreateEntity(reg_idx_, now_ms, sink);
          return;
        }
        slot.reg_state = EntityRegState::Error;
        slot.status_code = StatusCode::RosError;
        ++reg_idx_;
        reg_retries_ = 0U;
      }
    }

    state_ = SessionState::Connected;
    if (!notified_connected_) {
      notified_connected_ = true;
      if (on_connect_ != nullptr) {
        on_connect_(cb_ctx_);
      }
      if ((flags_ & kFlagNoTime) == 0U) {
        sendTimeRequest(now_ms, sink);
      }
    }
  }

  template <typename Sink>
  void handleFrame(const FrameView& view, uint32_t now_ms, Sink& sink) {
    if (!CommandCodec::isKnownCommand(view.type)) {
      return;
    }
    last_rx_ms_ = now_ms;
    const CommandType cmd_type = static_cast<CommandType>(view.type);

    switch (cmd_type) {
      case CommandType::Welcome: {
        WelcomeCmd cmd;
        if (CommandCodec::decodeWelcome(view.body, view.body_len, &cmd) != ParseStatus::Ok) {
          return;
        }
        if (cmd.proto_ver != kProtocolVersion || cmd.session != session_id_ ||
            cmd.status != StatusCode::Ok) {
          if (state_ != SessionState::Disconnected) {
            transitionToDisconnected(now_ms);
          }
          return;
        }
        agent_rx_max_ = cmd.rx_max;
        if (state_ == SessionState::Disconnected) {
          resetEntitiesToPending();
          state_ = SessionState::Registering;
          reg_idx_ = 0U;
          reg_retries_ = 0U;
          stepRegistration(now_ms, sink);
        }
        break;
      }

      case CommandType::Ping: {
        if (CommandCodec::decodeEmpty(view.body, view.body_len) == ParseStatus::Ok) {
          sendEmptyCommand(CommandType::Pong, view.seq, now_ms, sink);
        }
        break;
      }

      case CommandType::Pong: {
        (void)CommandCodec::decodeEmpty(view.body, view.body_len);
        break;
      }

      case CommandType::SyncRequest: {
        if (CommandCodec::decodeEmpty(view.body, view.body_len) == ParseStatus::Ok) {
          transitionToDisconnected(now_ms);
          sendHello(now_ms, sink);
        }
        break;
      }

      case CommandType::ResetSession: {
        ResetSessionCmd cmd;
        if (CommandCodec::decodeResetSession(view.body, view.body_len, &cmd) == ParseStatus::Ok) {
          transitionToDisconnected(now_ms);
        }
        break;
      }

      case CommandType::Status: {
        StatusCmd cmd;
        if (CommandCodec::decodeStatus(view.body, view.body_len, &cmd) != ParseStatus::Ok) {
          return;
        }
        if (cmd.id < max_entities_ && entities_[cmd.id].reg_state != EntityRegState::Unused) {
          EntitySlot& slot = entities_[cmd.id];
          slot.status_code = cmd.code;
          slot.reg_state =
              (cmd.code == StatusCode::Ok) ? EntityRegState::Active : EntityRegState::Error;
          if (state_ == SessionState::Registering && cmd.id == reg_idx_) {
            ++reg_idx_;
            reg_retries_ = 0U;
            stepRegistration(now_ms, sink);
          }
        }
        break;
      }

      case CommandType::Data: {
        if (state_ == SessionState::Disconnected) {
          return;
        }
        DataCmd cmd;
        if (CommandCodec::decodeData(view.body, view.body_len, &cmd) == ParseStatus::Ok) {
          if (cmd.id < max_entities_ &&
              entities_[cmd.id].kind == EntityKind::Subscriber &&
              entities_[cmd.id].reg_state == EntityRegState::Active &&
              entities_[cmd.id].on_data != nullptr) {
            entities_[cmd.id].on_data(entities_[cmd.id].user_ctx, cmd.cdr, cmd.cdr_len);
          }
        }
        break;
      }

      case CommandType::ServiceRequest: {
        if (state_ == SessionState::Disconnected) {
          return;
        }
        ServiceDataCmd cmd;
        if (CommandCodec::decodeServiceData(view.body, view.body_len, &cmd) == ParseStatus::Ok) {
          if (cmd.id < max_entities_ &&
              entities_[cmd.id].kind == EntityKind::ServiceServer &&
              entities_[cmd.id].reg_state == EntityRegState::Active &&
              entities_[cmd.id].on_service != nullptr) {
            entities_[cmd.id].on_service(entities_[cmd.id].user_ctx, cmd.req_id, cmd.cdr,
                                         cmd.cdr_len);
          }
        }
        break;
      }

      case CommandType::ServiceResponse: {
        if (state_ == SessionState::Disconnected) {
          return;
        }
        ServiceDataCmd cmd;
        if (CommandCodec::decodeServiceData(view.body, view.body_len, &cmd) == ParseStatus::Ok) {
          if (cmd.id < max_entities_ &&
              entities_[cmd.id].kind == EntityKind::ServiceClient &&
              entities_[cmd.id].reg_state == EntityRegState::Active &&
              entities_[cmd.id].on_service != nullptr) {
            entities_[cmd.id].on_service(entities_[cmd.id].user_ctx, cmd.req_id, cmd.cdr,
                                         cmd.cdr_len);
          }
        }
        break;
      }

      case CommandType::Time: {
        TimeCmd cmd;
        if (CommandCodec::decodeTime(view.body, view.body_len, &cmd) == ParseStatus::Ok) {
          const uint32_t half_rtt_ms =
              time_req_pending_ ? (static_cast<uint32_t>(now_ms - timer_ms_) / 2U) : 0U;
          time_req_pending_ = false;
          const uint32_t half_sec = half_rtt_ms / 1000U;
          const uint32_t half_rem_ms = half_rtt_ms - (half_sec * 1000U);
          uint32_t nsec = cmd.nanosec + (half_rem_ms * 1000000UL);
          int32_t sec = cmd.sec + static_cast<int32_t>(half_sec);
          if (nsec >= 1000000000UL) {
            nsec -= 1000000000UL;
            ++sec;
          }
          synced_time_.base_sec = sec;
          synced_time_.base_nanosec = nsec;
          synced_time_.sync_ms = now_ms;
          synced_time_.valid = true;
        }
        break;
      }

      default:
        break;
    }
  }

  FrameReader reader_;
  FrameWriter writer_;
  EntitySlot* entities_;
  ConnectCallback on_connect_;
  ConnectCallback on_disconnect_;
  void* cb_ctx_;

  uint32_t last_tx_ms_;
  uint32_t last_rx_ms_;
  uint32_t timer_ms_;
  SyncedTime synced_time_;

  uint16_t session_id_;
  uint16_t agent_rx_max_;
  StrView node_name_;
  StrView ns_;

  uint8_t max_entities_;
  SessionState state_;
  uint8_t flags_;
  uint8_t tx_seq_;
  uint8_t reg_idx_;
  uint8_t reg_retries_;
  uint8_t reg_sent_seq_;

  uint8_t hello_sent_once_ : 1;
  uint8_t time_req_pending_ : 1;
  uint8_t notified_connected_ : 1;
};

}  // namespace core
}  // namespace rosduino

#endif  // ROSDUINO_CORE_SESSION_H_
