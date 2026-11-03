// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_CORE_COMMANDS_H_
#define ROSDUINO_CORE_COMMANDS_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if defined(__AVR__)
#include <avr/pgmspace.h>
#endif

namespace rosduino {
namespace core {

static constexpr uint8_t kProtocolVersion = 1U;

/**
 * @brief Protocol command codes (`docs/protocol.md`, section 3).
 */
enum class CommandType : uint8_t {
  Hello = 0x01U,
  Welcome = 0x02U,
  Ping = 0x03U,
  Pong = 0x04U,
  SyncRequest = 0x05U,
  ResetSession = 0x06U,
  CreatePublisher = 0x10U,
  CreateSubscriber = 0x11U,
  CreateServiceServer = 0x12U,
  CreateServiceClient = 0x13U,
  DestroyEntity = 0x1FU,
  Status = 0x20U,
  Data = 0x30U,
  ServiceRequest = 0x31U,
  ServiceResponse = 0x32U,
  TimeRequest = 0x40U,
  Time = 0x41U,
  Log = 0x42U,
};

/**
 * @brief Protocol `status:u8` codes (`WELCOME` and `STATUS`).
 */
enum class StatusCode : uint8_t {
  Ok = 0U,
  UnknownType = 1U,
  HashMismatch = 2U,
  BadName = 3U,
  RosError = 4U,
  NoResources = 5U,
  UnknownEntity = 6U,
  TooLarge = 7U,
  VersionMismatch = 8U,
  NotInSession = 9U,
  Unsupported = 10U,
};

/**
 * @brief `RESET_SESSION` reason codes.
 */
enum class ResetReason : uint8_t {
  Unspecified = 0U,
  AgentShutdown = 1U,
  McuReboot = 2U,
  ProtocolError = 3U,
  Timeout = 4U,
};

/**
 * @brief `HELLO` flags bit definitions (`docs/protocol.md`, section 3.1).
 */
enum HelloFlags : uint8_t {
  kFlagProfileMask = 0x07U,
  kFlagProfileDefault = 0U,
  kFlagProfileLite = 1U,
  kFlagProfileSmall = 2U,
  kFlagProfileMedium = 3U,
  kFlagProfileLarge = 4U,
  kFlagNoTime = 0x08U,
  kFlagNoLog = 0x10U,
  kFlagAvrFloat64 = 0x20U,
};

/**
 * @brief `qos:u8` bit definitions (`docs/protocol.md`, section 3.4).
 */
enum QosBits : uint8_t {
  kQosBestEffort = 0x00U,
  kQosReliable = 0x01U,
  kQosVolatile = 0x00U,
  kQosTransientLocal = 0x02U,
};

/**
 * @brief `LOG` level codes matching `RCUTILS_LOG_SEVERITY`.
 */
enum class LogLevel : uint8_t {
  Debug = 10U,
  Info = 20U,
  Warn = 30U,
  Error = 40U,
  Fatal = 50U,
};

/**
 * @brief Result of parsing a command body.
 */
enum class ParseStatus : uint8_t {
  Ok = 0U,
  UnknownCommand,
  MalformedBody,
};

/**
 * @brief Non-null-terminated string view (`len:u8` + bytes) pointing into the frame buffer
 *        (or RAM/PROGMEM source when encoding).
 */
struct StrView {
  const char* data;
  uint8_t len;
  bool is_progmem;

  StrView() = default;
  constexpr StrView(const char* ptr, uint8_t length, bool progmem = false)
      : data(ptr), len(length), is_progmem(progmem) {}

  static inline StrView fromCString(const char* s) {
    if (s == nullptr) {
      return StrView(nullptr, 0U, false);
    }
    const size_t n = strlen(s);
    const uint8_t clamped = (n > 255U) ? 255U : static_cast<uint8_t>(n);
    return StrView(s, clamped, false);
  }

  bool equals(const char* cstr) const {
    if (cstr == nullptr) {
      return len == 0U;
    }
    const size_t n = strlen(cstr);
    if (n != len) {
      return false;
    }
    if (len == 0U) {
      return true;
    }
    return memcmp(data, cstr, len) == 0;
  }
};

struct HelloCmd {
  uint8_t proto_ver;
  uint16_t session;
  uint16_t rx_max;
  uint16_t tx_max;
  uint8_t max_entities;
  uint8_t flags;
  StrView node_name;
  StrView ns;

  constexpr HelloCmd()
      : proto_ver(kProtocolVersion),
        session(0U),
        rx_max(0U),
        tx_max(0U),
        max_entities(0U),
        flags(0U),
        node_name(),
        ns() {}
};

struct WelcomeCmd {
  uint8_t proto_ver;
  uint16_t session;
  uint16_t rx_max;
  StatusCode status;

  constexpr WelcomeCmd()
      : proto_ver(kProtocolVersion), session(0U), rx_max(0U), status(StatusCode::Ok) {}
  constexpr WelcomeCmd(uint8_t pv, uint16_t s, uint16_t rx, StatusCode st)
      : proto_ver(pv), session(s), rx_max(rx), status(st) {}
};

struct ResetSessionCmd {
  ResetReason reason;

  constexpr ResetSessionCmd() : reason(ResetReason::Unspecified) {}
  explicit constexpr ResetSessionCmd(ResetReason r) : reason(r) {}
};

struct CreateTopicCmd {
  uint8_t id;
  uint8_t qos;
  uint8_t depth;
  StrView name;
  StrView type;
  uint64_t hash;
  const uint8_t* hash_ptr;
  bool hash_in_progmem;

  constexpr CreateTopicCmd()
      : id(0U),
        qos(0U),
        depth(0U),
        name(),
        type(),
        hash(0ULL),
        hash_ptr(nullptr),
        hash_in_progmem(false) {}
};

struct CreateServiceCmd {
  uint8_t id;
  StrView name;
  StrView type;
  uint64_t hash;
  const uint8_t* hash_ptr;
  bool hash_in_progmem;

  constexpr CreateServiceCmd()
      : id(0U),
        name(),
        type(),
        hash(0ULL),
        hash_ptr(nullptr),
        hash_in_progmem(false) {}
};

struct DestroyEntityCmd {
  uint8_t id;

  constexpr DestroyEntityCmd() : id(0U) {}
  explicit constexpr DestroyEntityCmd(uint8_t entity_id) : id(entity_id) {}
};

struct StatusCmd {
  uint8_t ref_seq;
  uint8_t id;
  StatusCode code;

  constexpr StatusCmd() : ref_seq(0U), id(0U), code(StatusCode::Ok) {}
  constexpr StatusCmd(uint8_t rseq, uint8_t entity_id, StatusCode c)
      : ref_seq(rseq), id(entity_id), code(c) {}
};

struct DataCmd {
  uint8_t id;
  const uint8_t* cdr;
  size_t cdr_len;

  constexpr DataCmd() : id(0U), cdr(nullptr), cdr_len(0U) {}
};

struct ServiceDataCmd {
  uint8_t id;
  uint8_t req_id;
  const uint8_t* cdr;
  size_t cdr_len;

  constexpr ServiceDataCmd() : id(0U), req_id(0U), cdr(nullptr), cdr_len(0U) {}
};

struct TimeCmd {
  int32_t sec;
  uint32_t nanosec;

  constexpr TimeCmd() : sec(0), nanosec(0U) {}
  constexpr TimeCmd(int32_t s, uint32_t ns) : sec(s), nanosec(ns) {}
};

struct LogCmd {
  LogLevel level;
  StrView text;

  constexpr LogCmd() : level(LogLevel::Info), text() {}
  constexpr LogCmd(LogLevel lvl, StrView t) : level(lvl), text(t) {}
};

/**
 * @brief Command encoder and decoder (`docs/protocol.md`, section 3).
 *
 * Uses byte-level little-endian reads/writes without unaligned memory access.
 */
class CommandCodec {
 public:
  static inline bool isKnownCommand(uint8_t raw_type) {
    switch (static_cast<CommandType>(raw_type)) {
      case CommandType::Hello:
      case CommandType::Welcome:
      case CommandType::Ping:
      case CommandType::Pong:
      case CommandType::SyncRequest:
      case CommandType::ResetSession:
      case CommandType::CreatePublisher:
      case CommandType::CreateSubscriber:
      case CommandType::CreateServiceServer:
      case CommandType::CreateServiceClient:
      case CommandType::DestroyEntity:
      case CommandType::Status:
      case CommandType::Data:
      case CommandType::ServiceRequest:
      case CommandType::ServiceResponse:
      case CommandType::TimeRequest:
      case CommandType::Time:
      case CommandType::Log:
        return true;
      default:
        return false;
    }
  }

  // --- Encoders (return number of bytes written to `buf[0..cap)`, or 0 on overflow) ---

  static inline size_t encodeHello(const HelloCmd& cmd, uint8_t* buf, size_t cap,
                                   uint8_t (*read_byte_fn)(const char*, bool) = defaultReadByte) {
    const size_t total = 9U + 1U + cmd.node_name.len + 1U + cmd.ns.len;
    if (buf == nullptr || total > cap) {
      return 0U;
    }
    buf[0] = cmd.proto_ver;
    writeU16LE(&buf[1], cmd.session);
    writeU16LE(&buf[3], cmd.rx_max);
    writeU16LE(&buf[5], cmd.tx_max);
    buf[7] = cmd.max_entities;
    buf[8] = cmd.flags;
    size_t pos = 9U;
    pos += writeStr(&buf[pos], cmd.node_name, read_byte_fn);
    pos += writeStr(&buf[pos], cmd.ns, read_byte_fn);
    return pos;
  }

  static inline size_t encodeWelcome(const WelcomeCmd& cmd, uint8_t* buf, size_t cap) {
    if (buf == nullptr || cap < 6U) {
      return 0U;
    }
    buf[0] = cmd.proto_ver;
    writeU16LE(&buf[1], cmd.session);
    writeU16LE(&buf[3], cmd.rx_max);
    buf[5] = static_cast<uint8_t>(cmd.status);
    return 6U;
  }

  static inline size_t encodeResetSession(const ResetSessionCmd& cmd, uint8_t* buf, size_t cap) {
    if (buf == nullptr || cap < 1U) {
      return 0U;
    }
    buf[0] = static_cast<uint8_t>(cmd.reason);
    return 1U;
  }

  static inline size_t encodeCreateTopic(
      const CreateTopicCmd& cmd, uint8_t* buf, size_t cap,
      uint8_t (*read_byte_fn)(const char*, bool) = defaultReadByte) {
    const size_t total = 3U + 1U + cmd.name.len + 1U + cmd.type.len + 8U;
    if (buf == nullptr || total > cap) {
      return 0U;
    }
    buf[0] = cmd.id;
    buf[1] = cmd.qos;
    buf[2] = cmd.depth;
    return 3U + encodeNameTypeHash(&buf[3], cmd.name, cmd.type, cmd.hash, cmd.hash_ptr,
                                   cmd.hash_in_progmem, read_byte_fn);
  }

  static inline size_t encodeCreateService(
      const CreateServiceCmd& cmd, uint8_t* buf, size_t cap,
      uint8_t (*read_byte_fn)(const char*, bool) = defaultReadByte) {
    const size_t total = 1U + 1U + cmd.name.len + 1U + cmd.type.len + 8U;
    if (buf == nullptr || total > cap) {
      return 0U;
    }
    buf[0] = cmd.id;
    return 1U + encodeNameTypeHash(&buf[1], cmd.name, cmd.type, cmd.hash, cmd.hash_ptr,
                                   cmd.hash_in_progmem, read_byte_fn);
  }

  static inline size_t encodeDestroyEntity(const DestroyEntityCmd& cmd, uint8_t* buf, size_t cap) {
    if (buf == nullptr || cap < 1U) {
      return 0U;
    }
    buf[0] = cmd.id;
    return 1U;
  }

  static inline size_t encodeStatus(const StatusCmd& cmd, uint8_t* buf, size_t cap) {
    if (buf == nullptr || cap < 3U) {
      return 0U;
    }
    buf[0] = cmd.ref_seq;
    buf[1] = cmd.id;
    buf[2] = static_cast<uint8_t>(cmd.code);
    return 3U;
  }

  static inline size_t encodeDataHeader(uint8_t id, uint8_t* buf, size_t cap) {
    if (buf == nullptr || cap < 1U) {
      return 0U;
    }
    buf[0] = id;
    return 1U;
  }

  static inline size_t encodeServiceDataHeader(uint8_t id, uint8_t req_id, uint8_t* buf,
                                               size_t cap) {
    if (buf == nullptr || cap < 2U) {
      return 0U;
    }
    buf[0] = id;
    buf[1] = req_id;
    return 2U;
  }

  static inline size_t encodeTime(const TimeCmd& cmd, uint8_t* buf, size_t cap) {
    if (buf == nullptr || cap < 8U) {
      return 0U;
    }
    writeU32LE(&buf[0], static_cast<uint32_t>(cmd.sec));
    writeU32LE(&buf[4], cmd.nanosec);
    return 8U;
  }

  static inline size_t encodeLog(const LogCmd& cmd, uint8_t* buf, size_t cap,
                                 uint8_t (*read_byte_fn)(const char*, bool) = defaultReadByte) {
    const size_t total = 1U + 1U + cmd.text.len;
    if (buf == nullptr || total > cap) {
      return 0U;
    }
    buf[0] = static_cast<uint8_t>(cmd.level);
    size_t pos = 1U;
    pos += writeStr(&buf[pos], cmd.text, read_byte_fn);
    return pos;
  }

  // --- Decoders ---

  static inline ParseStatus decodeHello(const uint8_t* body, size_t len, HelloCmd* out) {
    if (body == nullptr || out == nullptr || len < 11U) {
      return ParseStatus::MalformedBody;
    }
    out->proto_ver = body[0];
    out->session = readU16LE(&body[1]);
    out->rx_max = readU16LE(&body[3]);
    out->tx_max = readU16LE(&body[5]);
    out->max_entities = body[7];
    out->flags = body[8];

    size_t pos = 9U;
    if (!readStr(body, len, &pos, &out->node_name)) {
      return ParseStatus::MalformedBody;
    }
    if (!readStr(body, len, &pos, &out->ns)) {
      return ParseStatus::MalformedBody;
    }
    return (pos == len) ? ParseStatus::Ok : ParseStatus::MalformedBody;
  }

  static inline ParseStatus decodeWelcome(const uint8_t* body, size_t len, WelcomeCmd* out) {
    if (body == nullptr || out == nullptr || len != 6U) {
      return ParseStatus::MalformedBody;
    }
    out->proto_ver = body[0];
    out->session = readU16LE(&body[1]);
    out->rx_max = readU16LE(&body[3]);
    out->status = static_cast<StatusCode>(body[5]);
    return ParseStatus::Ok;
  }

  static inline ParseStatus decodeEmpty(const uint8_t* /*body*/, size_t len) {
    return (len == 0U) ? ParseStatus::Ok : ParseStatus::MalformedBody;
  }

  static inline ParseStatus decodeResetSession(const uint8_t* body, size_t len,
                                               ResetSessionCmd* out) {
    if (body == nullptr || out == nullptr || len != 1U) {
      return ParseStatus::MalformedBody;
    }
    out->reason = static_cast<ResetReason>(body[0]);
    return ParseStatus::Ok;
  }

  static inline ParseStatus decodeCreateTopic(const uint8_t* body, size_t len,
                                              CreateTopicCmd* out) {
    if (body == nullptr || out == nullptr || len < 13U) {
      return ParseStatus::MalformedBody;
    }
    out->id = body[0];
    out->qos = body[1];
    out->depth = body[2];
    size_t pos = 3U;
    if (!readStr(body, len, &pos, &out->name)) {
      return ParseStatus::MalformedBody;
    }
    if (!readStr(body, len, &pos, &out->type)) {
      return ParseStatus::MalformedBody;
    }
    if (pos + 8U != len) {
      return ParseStatus::MalformedBody;
    }
    out->hash = readU64LE(&body[pos]);
    out->hash_ptr = &body[pos];
    out->hash_in_progmem = false;
    return ParseStatus::Ok;
  }

  static inline ParseStatus decodeCreateService(const uint8_t* body, size_t len,
                                                CreateServiceCmd* out) {
    if (body == nullptr || out == nullptr || len < 11U) {
      return ParseStatus::MalformedBody;
    }
    out->id = body[0];
    size_t pos = 1U;
    if (!readStr(body, len, &pos, &out->name)) {
      return ParseStatus::MalformedBody;
    }
    if (!readStr(body, len, &pos, &out->type)) {
      return ParseStatus::MalformedBody;
    }
    if (pos + 8U != len) {
      return ParseStatus::MalformedBody;
    }
    out->hash = readU64LE(&body[pos]);
    out->hash_ptr = &body[pos];
    out->hash_in_progmem = false;
    return ParseStatus::Ok;
  }

  static inline ParseStatus decodeDestroyEntity(const uint8_t* body, size_t len,
                                                DestroyEntityCmd* out) {
    if (body == nullptr || out == nullptr || len != 1U) {
      return ParseStatus::MalformedBody;
    }
    out->id = body[0];
    return ParseStatus::Ok;
  }

  static inline ParseStatus decodeStatus(const uint8_t* body, size_t len, StatusCmd* out) {
    if (body == nullptr || out == nullptr || len != 3U) {
      return ParseStatus::MalformedBody;
    }
    out->ref_seq = body[0];
    out->id = body[1];
    out->code = static_cast<StatusCode>(body[2]);
    return ParseStatus::Ok;
  }

  static inline ParseStatus decodeData(const uint8_t* body, size_t len, DataCmd* out) {
    if (body == nullptr || out == nullptr || len < 1U) {
      return ParseStatus::MalformedBody;
    }
    out->id = body[0];
    out->cdr = &body[1];
    out->cdr_len = len - 1U;
    return ParseStatus::Ok;
  }

  static inline ParseStatus decodeServiceData(const uint8_t* body, size_t len,
                                              ServiceDataCmd* out) {
    if (body == nullptr || out == nullptr || len < 2U) {
      return ParseStatus::MalformedBody;
    }
    out->id = body[0];
    out->req_id = body[1];
    out->cdr = &body[2];
    out->cdr_len = len - 2U;
    return ParseStatus::Ok;
  }

  static inline ParseStatus decodeTime(const uint8_t* body, size_t len, TimeCmd* out) {
    if (body == nullptr || out == nullptr || len != 8U) {
      return ParseStatus::MalformedBody;
    }
    out->sec = static_cast<int32_t>(readU32LE(&body[0]));
    out->nanosec = readU32LE(&body[4]);
    return ParseStatus::Ok;
  }

  static inline ParseStatus decodeLog(const uint8_t* body, size_t len, LogCmd* out) {
    if (body == nullptr || out == nullptr || len < 2U) {
      return ParseStatus::MalformedBody;
    }
    out->level = static_cast<LogLevel>(body[0]);
    size_t pos = 1U;
    if (!readStr(body, len, &pos, &out->text)) {
      return ParseStatus::MalformedBody;
    }
    return (pos == len) ? ParseStatus::Ok : ParseStatus::MalformedBody;
  }

  /**
   * @brief Validate that a raw frame `(type, body, len)` is a known command with a well-formed body.
   */
  static inline ParseStatus validateFrame(uint8_t raw_type, const uint8_t* body, size_t len) {
    if (!isKnownCommand(raw_type)) {
      return ParseStatus::UnknownCommand;
    }
    switch (static_cast<CommandType>(raw_type)) {
      case CommandType::Hello: {
        HelloCmd c{};
        return decodeHello(body, len, &c);
      }
      case CommandType::Welcome: {
        WelcomeCmd c{};
        return decodeWelcome(body, len, &c);
      }
      case CommandType::Ping:
      case CommandType::Pong:
      case CommandType::SyncRequest:
      case CommandType::TimeRequest:
        return decodeEmpty(body, len);
      case CommandType::ResetSession: {
        ResetSessionCmd c{};
        return decodeResetSession(body, len, &c);
      }
      case CommandType::CreatePublisher:
      case CommandType::CreateSubscriber: {
        CreateTopicCmd c{};
        return decodeCreateTopic(body, len, &c);
      }
      case CommandType::CreateServiceServer:
      case CommandType::CreateServiceClient: {
        CreateServiceCmd c{};
        return decodeCreateService(body, len, &c);
      }
      case CommandType::DestroyEntity: {
        DestroyEntityCmd c{};
        return decodeDestroyEntity(body, len, &c);
      }
      case CommandType::Status: {
        StatusCmd c{};
        return decodeStatus(body, len, &c);
      }
      case CommandType::Data: {
        DataCmd c{};
        return decodeData(body, len, &c);
      }
      case CommandType::ServiceRequest:
      case CommandType::ServiceResponse: {
        ServiceDataCmd c{};
        return decodeServiceData(body, len, &c);
      }
      case CommandType::Time: {
        TimeCmd c{};
        return decodeTime(body, len, &c);
      }
      case CommandType::Log: {
        LogCmd c{};
        return decodeLog(body, len, &c);
      }
    }
    return ParseStatus::UnknownCommand;
  }

 private:
  static inline uint8_t defaultReadByte(const char* ptr, bool is_progmem) {
#if defined(__AVR__)
    if (is_progmem) {
      return pgm_read_byte(ptr);
    }
#else
    (void)is_progmem;
#endif
    return static_cast<uint8_t>(*ptr);
  }

  static inline size_t encodeNameTypeHash(uint8_t* buf, const StrView& name, const StrView& type,
                                          uint64_t hash, const uint8_t* hash_ptr,
                                          bool hash_in_progmem,
                                          uint8_t (*read_byte_fn)(const char*, bool)) {
    size_t pos = 0U;
    pos += writeStr(&buf[pos], name, read_byte_fn);
    pos += writeStr(&buf[pos], type, read_byte_fn);
    if (hash_ptr != nullptr) {
      for (uint8_t i = 0U; i < 8U; ++i) {
        buf[pos + i] =
            read_byte_fn(reinterpret_cast<const char*>(&hash_ptr[i]), hash_in_progmem);
      }
    } else {
#if defined(__AVR__)
      (void)hash;
      for (uint8_t i = 0U; i < 8U; ++i) {
        buf[pos + i] = 0U;
      }
#else
      writeU64LE(&buf[pos], hash);
#endif
    }
    return pos + 8U;
  }

  static inline void writeU16LE(uint8_t* dst, uint16_t v) {
    dst[0] = static_cast<uint8_t>(v & 0xFFU);
    dst[1] = static_cast<uint8_t>((v >> 8U) & 0xFFU);
  }

  static inline void writeU32LE(uint8_t* dst, uint32_t v) {
    dst[0] = static_cast<uint8_t>(v & 0xFFU);
    dst[1] = static_cast<uint8_t>((v >> 8U) & 0xFFU);
    dst[2] = static_cast<uint8_t>((v >> 16U) & 0xFFU);
    dst[3] = static_cast<uint8_t>((v >> 24U) & 0xFFU);
  }

  static inline void writeU64LE(uint8_t* dst, uint64_t v) {
    for (uint8_t i = 0U; i < 8U; ++i) {
      dst[i] = static_cast<uint8_t>((v >> (8U * i)) & 0xFFU);
    }
  }

  static inline uint16_t readU16LE(const uint8_t* src) {
    return static_cast<uint16_t>(src[0]) | (static_cast<uint16_t>(src[1]) << 8U);
  }

  static inline uint32_t readU32LE(const uint8_t* src) {
    return static_cast<uint32_t>(src[0]) |
           (static_cast<uint32_t>(src[1]) << 8U) |
           (static_cast<uint32_t>(src[2]) << 16U) |
           (static_cast<uint32_t>(src[3]) << 24U);
  }

  static inline uint64_t readU64LE(const uint8_t* src) {
    uint64_t v = 0ULL;
    for (uint8_t i = 0U; i < 8U; ++i) {
      v |= static_cast<uint64_t>(src[i]) << (8U * i);
    }
    return v;
  }

  static inline size_t writeStr(uint8_t* dst, const StrView& s,
                                uint8_t (*read_byte_fn)(const char*, bool)) {
    dst[0] = s.len;
    for (uint8_t i = 0U; i < s.len; ++i) {
      dst[1U + i] = read_byte_fn(s.data + i, s.is_progmem);
    }
    return 1U + static_cast<size_t>(s.len);
  }

  static inline bool readStr(const uint8_t* body, size_t body_len, size_t* pos, StrView* out) {
    if (*pos >= body_len) {
      return false;
    }
    const uint8_t slen = body[(*pos)++];
    if (*pos + slen > body_len) {
      return false;
    }
    out->data = reinterpret_cast<const char*>(&body[*pos]);
    out->len = slen;
    out->is_progmem = false;
    *pos += slen;
    return true;
  }
};

}  // namespace core
}  // namespace rosduino

#endif  // ROSDUINO_CORE_COMMANDS_H_
