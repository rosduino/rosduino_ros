// SPDX-License-Identifier: Apache-2.0
#include "bridge/agent_session.h"
#include "bridge/qos.h"

#include <cctype>
#include <chrono>
#include <cstring>

namespace rosduino {
namespace host {
namespace {

std::string strToString(const core::StrView& v) {
  if (v.data == nullptr || v.len == 0U) {
    return std::string();
  }
  return std::string(v.data, v.data + v.len);
}

bool isNameChar(char c) {
  return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
}

bool isValidNodeName(const std::string& name) {
  if (name.empty()) {
    return false;
  }
  if (!(std::isalpha(static_cast<unsigned char>(name[0])) != 0 || name[0] == '_')) {
    return false;
  }
  for (char c : name) {
    if (!isNameChar(c)) {
      return false;
    }
  }
  return true;
}

bool isValidNamespace(const std::string& ns) {
  if (ns.empty() || ns == "/") {
    return true;
  }
  if (ns.front() != '/' || ns.back() == '/') {
    return false;
  }
  size_t i = 1U;
  while (i < ns.size()) {
    if (!(std::isalpha(static_cast<unsigned char>(ns[i])) != 0 || ns[i] == '_')) {
      return false;
    }
    ++i;
    while (i < ns.size() && isNameChar(ns[i])) {
      ++i;
    }
    if (i < ns.size()) {
      if (ns[i] != '/') {
        return false;
      }
      ++i;
    }
  }
  return true;
}

bool isValidTopicName(const std::string& name) {
  if (name.empty() || name == "/") {
    return false;
  }
  const size_t start = (name.front() == '/') ? 1U : 0U;
  if (start >= name.size()) {
    return false;
  }
  size_t i = start;
  while (i < name.size()) {
    if (!(std::isalpha(static_cast<unsigned char>(name[i])) != 0 || name[i] == '_')) {
      return false;
    }
    ++i;
    while (i < name.size() && isNameChar(name[i])) {
      ++i;
    }
    if (i < name.size()) {
      if (name[i] != '/') {
        return false;
      }
      ++i;
      if (i >= name.size()) {
        return false;
      }
    }
  }
  return true;
}

void defaultTime(int32_t* sec, uint32_t* nsec) {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  *sec = static_cast<int32_t>(ns / 1000000000LL);
  *nsec = static_cast<uint32_t>(ns % 1000000000LL);
}

}  // namespace

AgentSession::AgentSession(RosBackend& backend, AgentConfig cfg)
    : backend_(backend),
      cfg_(cfg),
      time_source_(defaultTime),
      rx_buf_(cfg_.rx_max, 0U),
      tx_buf_(cfg_.rx_max, 0U),
      reader_(rx_buf_.data(), rx_buf_.size()),
      writer_(tx_buf_.data(), tx_buf_.size()),
      state_(AgentState::WaitingHello),
      max_entities_(0U),
      flags_(0U),
      peer_rx_max_(0U),
      tx_seq_(0U),
      last_rx_ms_(0U),
      last_sync_ms_(0U),
      last_now_ms_(0U),
      last_pong_ms_(0U),
      tx_frames_(0U),
      no_resources_(0U),
      too_large_(0U),
      proto_ver_(0U),
      sync_sent_(false),
      last_pong_ms_cache_(0U),
      last_now_ms_cache_(0U) {
  backend_.setIncomingHandler([this](uint8_t id, const uint8_t* cdr, size_t len) {
    IncomingData msg;
    msg.id = id;
    if (cdr != nullptr && len > 0U) {
      msg.cdr.assign(cdr, cdr + len);
    }
    std::lock_guard<std::mutex> lock(incoming_mu_);
    incoming_.push_back(std::move(msg));
  });
  backend_.setServiceToMcuHandler(
      [this](uint8_t id, uint8_t req_id, const uint8_t* cdr, size_t len) {
        IncomingService msg;
        msg.id = id;
        msg.req_id = req_id;
        if (cdr != nullptr && len > 0U) {
          msg.cdr.assign(cdr, cdr + len);
        }
        std::lock_guard<std::mutex> lock(service_to_mcu_mu_);
        service_to_mcu_.push_back(std::move(msg));
      });
  backend_.setServiceFromRosHandler(
      [this](uint8_t id, uint8_t req_id, const uint8_t* cdr, size_t len) {
        IncomingService msg;
        msg.id = id;
        msg.req_id = req_id;
        if (cdr != nullptr && len > 0U) {
          msg.cdr.assign(cdr, cdr + len);
        }
        std::lock_guard<std::mutex> lock(service_from_ros_mu_);
        service_from_ros_.push_back(std::move(msg));
      });
  refreshCache();
}

AgentSession::~AgentSession() {
  backend_.setIncomingHandler(nullptr);
  backend_.setServiceToMcuHandler(nullptr);
  backend_.setServiceFromRosHandler(nullptr);
}

uint8_t AgentSession::nextSeq() {
  const uint8_t s = tx_seq_;
  tx_seq_ = static_cast<uint8_t>(tx_seq_ + 1U);
  return s;
}

void AgentSession::enqueueEmpty(core::CommandType type) {
  queue_.push(static_cast<uint8_t>(type), nextSeq(), nullptr, 0U);
}

void AgentSession::enqueueWelcome(uint16_t session, core::StatusCode status, uint8_t seq) {
  uint8_t body[6] = {};
  const core::WelcomeCmd cmd{core::kProtocolVersion, session, cfg_.rx_max, status};
  const size_t n = core::CommandCodec::encodeWelcome(cmd, body, sizeof(body));
  queue_.push(static_cast<uint8_t>(core::CommandType::Welcome), seq, body, n);
}

void AgentSession::enqueueStatus(uint8_t ref_seq, uint8_t id, core::StatusCode code) {
  recordStatus(id, code);
  uint8_t body[3] = {};
  const core::StatusCmd cmd{ref_seq, id, code};
  const size_t n = core::CommandCodec::encodeStatus(cmd, body, sizeof(body));
  queue_.push(static_cast<uint8_t>(core::CommandType::Status), nextSeq(), body, n);
}

void AgentSession::recordStatus(uint8_t id, core::StatusCode code) {
  if (code == core::StatusCode::NoResources) {
    ++no_resources_;
  } else if (code == core::StatusCode::TooLarge) {
    ++too_large_;
  }
  if (id < entities_.size() && entities_[id].occupied) {
    entities_[id].last_status = code;
  }
}

void AgentSession::refreshCache() {
  std::lock_guard<std::mutex> lock(snap_mu_);
  cache_ = AgentSnapshot{};
  cache_.state = state_;
  cache_.active = (state_ == AgentState::Active);
  cache_.node_name = node_name_;
  cache_.proto_ver = proto_ver_;
  cache_.flags = flags_;
  cache_.mcu_profile = static_cast<uint8_t>(flags_ & core::kFlagProfileMask);
  cache_.max_entities = max_entities_;
  for (const Entity& e : entities_) {
    if (!e.occupied) {
      continue;
    }
    ++cache_.entity_count;
    switch (e.kind) {
      case EntityKind::Publisher:
        ++cache_.publishers;
        break;
      case EntityKind::Subscriber:
        ++cache_.subscribers;
        break;
      case EntityKind::ServiceServer:
        ++cache_.service_servers;
        break;
      case EntityKind::ServiceClient:
        ++cache_.service_clients;
        break;
      default:
        break;
    }
    if (e.last_status != core::StatusCode::Ok) {
      ++cache_.entity_errors;
    }
  }
  const core::FrameStats& st = reader_.stats();
  cache_.rx_frames = st.frames_received;
  cache_.tx_frames = tx_frames_;
  cache_.crc_errors = static_cast<uint32_t>(st.crc_errors);
  cache_.cobs_errors = static_cast<uint32_t>(st.cobs_errors);
  cache_.no_resources = no_resources_;
  cache_.too_large = too_large_;
  cache_.losses = static_cast<uint32_t>(st.overflow_errors) +
                  static_cast<uint32_t>(st.too_short_errors) +
                  static_cast<uint32_t>(st.timeout_errors) + no_resources_;
  last_pong_ms_cache_ = last_pong_ms_;
  last_now_ms_cache_ = last_now_ms_;
}

AgentSnapshot AgentSession::snapshot() const {
  std::lock_guard<std::mutex> lock(snap_mu_);
  AgentSnapshot snap = cache_;
  if (last_pong_ms_cache_ == 0U) {
    snap.last_pong_age_ms = 0xFFFFFFFFU;
  } else {
    snap.last_pong_age_ms = last_now_ms_cache_ - last_pong_ms_cache_;
  }
  return snap;
}

void AgentSession::resizeEntities(uint8_t max_entities) {
  entities_.assign(max_entities, Entity{});
  max_entities_ = max_entities;
}

void AgentSession::dropSession() {
  backend_.destroyAllEntities();
  backend_.destroyNode();
  entities_.clear();
  max_entities_ = 0U;
  session_.reset();
  peer_rx_max_ = 0U;
  state_ = AgentState::WaitingHello;
  sync_sent_ = false;
  {
    std::lock_guard<std::mutex> lock(incoming_mu_);
    incoming_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(service_to_mcu_mu_);
    service_to_mcu_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(service_from_ros_mu_);
    service_from_ros_.clear();
  }
  pending_mcu_services_.clear();
}

void AgentSession::onRxTimeout() {
  if (!cfg_.keep_on_disconnect) {
    dropSession();
  } else {
    state_ = AgentState::WaitingHello;
    sync_sent_ = false;
  }
}

void AgentSession::handleHello(const core::HelloCmd& cmd, uint8_t seq) {
  if (cmd.proto_ver != core::kProtocolVersion) {
    enqueueWelcome(cmd.session, core::StatusCode::VersionMismatch, seq);
    return;
  }
  const std::string node_name = strToString(cmd.node_name);
  const std::string ns = strToString(cmd.ns);
  if (!isValidNodeName(node_name) || !isValidNamespace(ns)) {
    enqueueWelcome(cmd.session, core::StatusCode::BadName, seq);
    return;
  }

  const bool new_session = !session_.has_value() || session_.value() != cmd.session;
  if (new_session) {
    if (session_.has_value()) {
      backend_.destroyAllEntities();
      backend_.destroyNode();
    }
    const std::string actual = backend_.createNode(node_name, ns);
    if (actual.empty()) {
      enqueueWelcome(cmd.session, core::StatusCode::RosError, seq);
      return;
    }
    resizeEntities(cmd.max_entities);
    session_ = cmd.session;
    node_name_ = actual;
    proto_ver_ = cmd.proto_ver;
  }

  flags_ = cmd.flags;
  peer_rx_max_ = cmd.rx_max;
  state_ = AgentState::Active;
  enqueueWelcome(cmd.session, core::StatusCode::Ok, seq);
}

void AgentSession::handleCreateTopic(const core::CreateTopicCmd& cmd, uint8_t seq,
                                     EntityKind kind) {
  if (cmd.id >= max_entities_) {
    enqueueStatus(seq, cmd.id, core::StatusCode::NoResources);
    return;
  }
  const std::string name = strToString(cmd.name);
  const std::string type = strToString(cmd.type);
  if (!isValidTopicName(name) || type.empty()) {
    enqueueStatus(seq, cmd.id, core::StatusCode::BadName);
    return;
  }
  const uint8_t depth = decodeTopicQos(cmd.qos, cmd.depth).history_depth;

  Entity& slot = entities_[cmd.id];
  const bool hash_same = (cmd.hash == 0ULL) || (slot.hash == cmd.hash);
  if (slot.occupied && slot.kind == kind && slot.name == name && slot.type == type && hash_same &&
      slot.qos == cmd.qos && slot.depth == depth) {
    enqueueStatus(seq, cmd.id, core::StatusCode::Ok);
    return;
  }
  if (slot.occupied) {
    backend_.destroyEntity(cmd.id);
    slot = Entity{};
  }

  const std::optional<uint64_t> env_hash = backend_.lookupTypeHash(type);
  if (!env_hash.has_value()) {
    enqueueStatus(seq, cmd.id, core::StatusCode::UnknownType);
    return;
  }
  if (cmd.hash != 0ULL && *env_hash != 0ULL && cmd.hash != *env_hash) {
    enqueueStatus(seq, cmd.id, core::StatusCode::HashMismatch);
    return;
  }

  TopicSpec spec;
  spec.name = name;
  spec.type = type;
  spec.hash = (cmd.hash == 0ULL) ? *env_hash : cmd.hash;
  spec.qos = cmd.qos;
  spec.depth = depth;
  const core::StatusCode code = (kind == EntityKind::Subscriber)
                                    ? backend_.createSubscriber(cmd.id, spec)
                                    : backend_.createPublisher(cmd.id, spec);
  if (code == core::StatusCode::Ok) {
    slot.kind = kind;
    slot.occupied = true;
    slot.name = name;
    slot.type = type;
    slot.hash = spec.hash;
    slot.qos = cmd.qos;
    slot.depth = depth;
  }
  enqueueStatus(seq, cmd.id, code);
}

void AgentSession::handleCreateService(const core::CreateServiceCmd& cmd, uint8_t seq,
                                       EntityKind kind) {
  if (cmd.id >= max_entities_) {
    enqueueStatus(seq, cmd.id, core::StatusCode::NoResources);
    return;
  }
  const std::string name = strToString(cmd.name);
  const std::string type = strToString(cmd.type);
  if (!isValidTopicName(name) || type.empty()) {
    enqueueStatus(seq, cmd.id, core::StatusCode::BadName);
    return;
  }

  Entity& slot = entities_[cmd.id];
  const bool hash_same = (cmd.hash == 0ULL) || (slot.hash == cmd.hash);
  if (slot.occupied && slot.kind == kind && slot.name == name && slot.type == type && hash_same) {
    enqueueStatus(seq, cmd.id, slot.last_status);
    return;
  }
  if (slot.occupied) {
    backend_.destroyEntity(cmd.id);
    slot = Entity{};
  }

  const std::optional<uint64_t> env_hash = backend_.lookupTypeHash(type);
  if (!env_hash.has_value()) {
    enqueueStatus(seq, cmd.id, core::StatusCode::UnknownType);
    return;
  }
  if (cmd.hash != 0ULL && *env_hash != 0ULL && cmd.hash != *env_hash) {
    enqueueStatus(seq, cmd.id, core::StatusCode::HashMismatch);
    return;
  }

  ServiceSpec spec;
  spec.name = name;
  spec.type = type;
  spec.hash = (cmd.hash == 0ULL) ? *env_hash : cmd.hash;
  const core::StatusCode code = (kind == EntityKind::ServiceClient)
                                    ? backend_.createServiceClient(cmd.id, spec)
                                    : backend_.createServiceServer(cmd.id, spec);
  if (code == core::StatusCode::Ok || code == core::StatusCode::Unsupported) {
    slot.kind = kind;
    slot.occupied = true;
    slot.name = name;
    slot.type = type;
    slot.hash = spec.hash;
  }
  enqueueStatus(seq, cmd.id, code);
}

void AgentSession::drainIncoming() {
  std::vector<IncomingData> local;
  {
    std::lock_guard<std::mutex> lock(incoming_mu_);
    local.swap(incoming_);
  }
  for (const IncomingData& msg : local) {
    if (state_ != AgentState::Active) {
      continue;
    }
    if (msg.id >= max_entities_ || !entities_[msg.id].occupied ||
        entities_[msg.id].kind != EntityKind::Subscriber) {
      continue;
    }
    std::vector<uint8_t> body(1U + msg.cdr.size());
    body[0] = msg.id;
    if (!msg.cdr.empty()) {
      std::memcpy(body.data() + 1U, msg.cdr.data(), msg.cdr.size());
    }
    queue_.push(static_cast<uint8_t>(core::CommandType::Data), nextSeq(), body.data(),
                body.size());
  }
}

void AgentSession::drainServiceToMcu(uint32_t now_ms) {
  std::vector<IncomingService> local;
  {
    std::lock_guard<std::mutex> lock(service_to_mcu_mu_);
    local.swap(service_to_mcu_);
  }
  for (const IncomingService& msg : local) {
    if (state_ != AgentState::Active) {
      continue;
    }
    if (msg.id >= max_entities_ || !entities_[msg.id].occupied ||
        entities_[msg.id].kind != EntityKind::ServiceServer) {
      continue;
    }
    std::vector<uint8_t> body(2U + msg.cdr.size());
    body[0] = msg.id;
    body[1] = msg.req_id;
    if (!msg.cdr.empty()) {
      std::memcpy(body.data() + 2U, msg.cdr.data(), msg.cdr.size());
    }
    queue_.push(static_cast<uint8_t>(core::CommandType::ServiceRequest), nextSeq(), body.data(),
                body.size());
    PendingMcuService pending;
    pending.id = msg.id;
    pending.req_id = msg.req_id;
    pending.deadline_ms = now_ms + cfg_.service_timeout_ms;
    pending_mcu_services_.push_back(pending);
  }
}

void AgentSession::drainServiceFromRos() {
  std::vector<IncomingService> local;
  {
    std::lock_guard<std::mutex> lock(service_from_ros_mu_);
    local.swap(service_from_ros_);
  }
  for (const IncomingService& msg : local) {
    if (state_ != AgentState::Active) {
      continue;
    }
    if (msg.id >= max_entities_ || !entities_[msg.id].occupied ||
        entities_[msg.id].kind != EntityKind::ServiceClient) {
      continue;
    }
    std::vector<uint8_t> body(2U + msg.cdr.size());
    body[0] = msg.id;
    body[1] = msg.req_id;
    if (!msg.cdr.empty()) {
      std::memcpy(body.data() + 2U, msg.cdr.data(), msg.cdr.size());
    }
    queue_.push(static_cast<uint8_t>(core::CommandType::ServiceResponse), nextSeq(), body.data(),
                body.size());
  }
}

void AgentSession::expireServiceRequests(uint32_t now_ms) {
  std::vector<PendingMcuService> still;
  still.reserve(pending_mcu_services_.size());
  for (const PendingMcuService& pending : pending_mcu_services_) {
    if (static_cast<int32_t>(now_ms - pending.deadline_ms) < 0) {
      still.push_back(pending);
      continue;
    }
    backend_.completeServiceResponse(pending.id, pending.req_id, nullptr, 0U);
  }
  pending_mcu_services_.swap(still);
}

void AgentSession::handleDestroy(uint8_t id, uint8_t seq) {
  if (id >= max_entities_ || !entities_[id].occupied) {
    enqueueStatus(seq, id, core::StatusCode::UnknownEntity);
    return;
  }
  backend_.destroyEntity(id);
  entities_[id] = Entity{};
  enqueueStatus(seq, id, core::StatusCode::Ok);
}

void AgentSession::handleServiceRequest(const core::ServiceDataCmd& cmd) {
  if (cmd.id >= max_entities_ || !entities_[cmd.id].occupied ||
      entities_[cmd.id].kind != EntityKind::ServiceClient) {
    return;
  }
  (void)backend_.callService(cmd.id, cmd.req_id, cmd.cdr, cmd.cdr_len);
}

void AgentSession::handleServiceResponse(const core::ServiceDataCmd& cmd) {
  if (cmd.id >= max_entities_ || !entities_[cmd.id].occupied ||
      entities_[cmd.id].kind != EntityKind::ServiceServer) {
    return;
  }
  std::vector<PendingMcuService> still;
  still.reserve(pending_mcu_services_.size());
  bool matched = false;
  for (const PendingMcuService& pending : pending_mcu_services_) {
    if (!matched && pending.id == cmd.id && pending.req_id == cmd.req_id) {
      matched = true;
      continue;
    }
    still.push_back(pending);
  }
  pending_mcu_services_.swap(still);
  if (!matched) {
    return;
  }
  backend_.completeServiceResponse(cmd.id, cmd.req_id, cmd.cdr, cmd.cdr_len);
}

void AgentSession::handleData(const core::DataCmd& cmd) {
  if (cmd.id >= max_entities_ || !entities_[cmd.id].occupied ||
      entities_[cmd.id].kind != EntityKind::Publisher) {
    return;
  }
  (void)backend_.publish(cmd.id, cmd.cdr, cmd.cdr_len);
}

void AgentSession::handleFrame(const core::FrameView& view, uint32_t now_ms) {
  if (!core::CommandCodec::isKnownCommand(view.type)) {
    return;
  }
  const core::CommandType type = static_cast<core::CommandType>(view.type);

  if (type == core::CommandType::Hello) {
    core::HelloCmd hello;
    if (core::CommandCodec::decodeHello(view.body, view.body_len, &hello) ==
        core::ParseStatus::Ok) {
      handleHello(hello, view.seq);
    }
    return;
  }

  if (type == core::CommandType::ResetSession) {
    core::ResetSessionCmd reset;
    if (core::CommandCodec::decodeResetSession(view.body, view.body_len, &reset) ==
        core::ParseStatus::Ok) {
      dropSession();
    }
    return;
  }

  auto notInSessionStatus = [&](uint8_t id) {
    enqueueStatus(view.seq, id, core::StatusCode::NotInSession);
  };

  if (state_ != AgentState::Active) {
    switch (type) {
      case core::CommandType::CreatePublisher:
      case core::CommandType::CreateSubscriber:
      case core::CommandType::CreateServiceServer:
      case core::CommandType::CreateServiceClient:
      case core::CommandType::DestroyEntity:
      case core::CommandType::Data:
      case core::CommandType::ServiceRequest:
      case core::CommandType::ServiceResponse: {
        const uint8_t id = (view.body_len >= 1U) ? view.body[0] : 0U;
        notInSessionStatus(id);
        break;
      }
      default:
        break;
    }
    return;
  }

  switch (type) {
    case core::CommandType::CreatePublisher:
    case core::CommandType::CreateSubscriber: {
      core::CreateTopicCmd cmd;
      if (core::CommandCodec::decodeCreateTopic(view.body, view.body_len, &cmd) ==
          core::ParseStatus::Ok) {
        const EntityKind kind = (type == core::CommandType::CreateSubscriber)
                                    ? EntityKind::Subscriber
                                    : EntityKind::Publisher;
        handleCreateTopic(cmd, view.seq, kind);
      }
      break;
    }
    case core::CommandType::CreateServiceServer:
    case core::CommandType::CreateServiceClient: {
      core::CreateServiceCmd cmd;
      if (core::CommandCodec::decodeCreateService(view.body, view.body_len, &cmd) ==
          core::ParseStatus::Ok) {
        const EntityKind kind = (type == core::CommandType::CreateServiceClient)
                                    ? EntityKind::ServiceClient
                                    : EntityKind::ServiceServer;
        handleCreateService(cmd, view.seq, kind);
      }
      break;
    }
    case core::CommandType::DestroyEntity: {
      core::DestroyEntityCmd cmd;
      if (core::CommandCodec::decodeDestroyEntity(view.body, view.body_len, &cmd) ==
          core::ParseStatus::Ok) {
        handleDestroy(cmd.id, view.seq);
      }
      break;
    }
    case core::CommandType::Data: {
      core::DataCmd cmd;
      if (core::CommandCodec::decodeData(view.body, view.body_len, &cmd) == core::ParseStatus::Ok) {
        handleData(cmd);
      }
      break;
    }
    case core::CommandType::ServiceRequest: {
      core::ServiceDataCmd cmd;
      if (core::CommandCodec::decodeServiceData(view.body, view.body_len, &cmd) ==
          core::ParseStatus::Ok) {
        handleServiceRequest(cmd);
      }
      break;
    }
    case core::CommandType::ServiceResponse: {
      core::ServiceDataCmd cmd;
      if (core::CommandCodec::decodeServiceData(view.body, view.body_len, &cmd) ==
          core::ParseStatus::Ok) {
        handleServiceResponse(cmd);
      }
      break;
    }
    case core::CommandType::Ping: {
      if (core::CommandCodec::decodeEmpty(view.body, view.body_len) == core::ParseStatus::Ok) {
        last_pong_ms_ = now_ms;
        queue_.push(static_cast<uint8_t>(core::CommandType::Pong), view.seq, nullptr, 0U);
      }
      break;
    }
    case core::CommandType::TimeRequest: {
      if (core::CommandCodec::decodeEmpty(view.body, view.body_len) == core::ParseStatus::Ok) {
        int32_t sec = 0;
        uint32_t nsec = 0U;
        if (time_source_) {
          time_source_(&sec, &nsec);
        }
        uint8_t body[8] = {};
        const core::TimeCmd time_cmd{sec, nsec};
        const size_t n = core::CommandCodec::encodeTime(time_cmd, body, sizeof(body));
        queue_.push(static_cast<uint8_t>(core::CommandType::Time), view.seq, body, n);
      }
      break;
    }
    case core::CommandType::Log: {
      core::LogCmd cmd;
      if (core::CommandCodec::decodeLog(view.body, view.body_len, &cmd) == core::ParseStatus::Ok) {
        backend_.log(cmd.level, strToString(cmd.text));
      }
      break;
    }
    default:
      break;
  }
}

}  // namespace host
}  // namespace rosduino
