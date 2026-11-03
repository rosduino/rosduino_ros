// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_CORE_FRAME_H_
#define ROSDUINO_CORE_FRAME_H_

#include <stddef.h>
#include <stdint.h>

#include "rosduino/core/cobs.h"
#include "rosduino/core/crc16.h"

namespace rosduino {
namespace core {

#if defined(__AVR__)
using FrameSize = uint16_t;
using ErrorCounter = uint8_t;
#else
using FrameSize = size_t;
using ErrorCounter = uint32_t;
#endif

/**
 * @brief Status returned by `FrameReader::pushByte`.
 */
enum class FrameStatus : uint8_t {
  Incomplete = 0,     ///< Waiting for more bytes or 0x00 delimiter
  FrameReady,         ///< Valid frame decoded and CRC-verified
  EmptyDelimiter,     ///< Consecutive 0x00 delimiter ignored (no error)
  CrcMismatch,        ///< CRC-16 check failed
  CobsError,          ///< Malformed/truncated COBS block
  TooShort,           ///< Decoded frame < 4 bytes (type + seq + crc16)
  FrameTooLarge,      ///< Decoded frame exceeds rx_max
};

/**
 * @brief Decoded frame view pointing into `FrameReader`'s buffer.
 */
struct FrameView {
  uint8_t type;
  uint8_t seq;
  const uint8_t* body;
  size_t body_len;
  uint16_t crc16;

  FrameView() = default;
};

/**
 * @brief Cumulative diagnostic counters for `FrameReader`.
 */
struct FrameStats {
  ErrorCounter frames_received;
  ErrorCounter crc_errors;
  ErrorCounter cobs_errors;
  ErrorCounter too_short_errors;
  ErrorCounter overflow_errors;
  ErrorCounter timeout_errors;

  FrameStats() = default;

  uint32_t totalErrors() const {
    return static_cast<uint32_t>(crc_errors) + static_cast<uint32_t>(cobs_errors) +
           static_cast<uint32_t>(too_short_errors) + static_cast<uint32_t>(overflow_errors) +
           static_cast<uint32_t>(timeout_errors);
  }
};

/**
 * @brief Byte-by-byte COBS frame decoder with inter-byte timeout, overflow protection,
 *        and CRC-16/CCITT-FALSE verification.
 *
 * Decodes COBS on-the-fly directly into a user-supplied buffer of size `rx_max`
 * (the maximum decoded frame size, including 4 bytes of type + seq + crc16).
 * No secondary buffer or heap allocation is required.
 */
class FrameReader {
 public:
  static constexpr uint16_t kDefaultInterByteTimeoutMs = 50U;
  static constexpr size_t kMinDecodedFrameSize = 4U;

  FrameReader() = default;

  FrameReader(uint8_t* buf, size_t rx_max,
              uint16_t inter_byte_timeout_ms = kDefaultInterByteTimeoutMs)
      : buf_(buf),
        rx_max_(static_cast<FrameSize>(rx_max)),
        write_idx_(0U),
        timeout_ms_(inter_byte_timeout_ms),
        last_byte_ms_(0U),
        code_(0U),
        remaining_in_block_(0U),
        in_frame_(false),
        overflowed_(false),
        stats_() {}

  void init(uint8_t* buf, size_t rx_max,
            uint16_t inter_byte_timeout_ms = kDefaultInterByteTimeoutMs) {
    buf_ = buf;
    rx_max_ = static_cast<FrameSize>(rx_max);
    timeout_ms_ = inter_byte_timeout_ms;
    reset();
  }

  void reset() {
    write_idx_ = 0U;
    code_ = 0U;
    remaining_in_block_ = 0U;
    in_frame_ = false;
    overflowed_ = false;
  }

  /**
   * @brief Check if an incomplete frame has timed out before any new byte arrives.
   * @return true if an incomplete frame was discarded due to timeout.
   */
  bool checkTimeout(uint32_t now_ms) {
    if (in_frame_ && timeout_ms_ > 0U &&
        static_cast<uint32_t>(now_ms - last_byte_ms_) > timeout_ms_) {
      ++stats_.timeout_errors;
      reset();
      return true;
    }
    return false;
  }

  /**
   * @brief Feed one wire byte into the frame reader.
   *
   * @param byte      Incoming wire byte.
   * @param now_ms    Current monotonic timestamp in milliseconds.
   * @param out_frame Populated when `FrameStatus::FrameReady` is returned.
   */
  FrameStatus pushByte(uint8_t byte, uint32_t now_ms, FrameView* out_frame) {
    if (in_frame_ && timeout_ms_ > 0U &&
        static_cast<uint32_t>(now_ms - last_byte_ms_) > timeout_ms_) {
      ++stats_.timeout_errors;
      reset();
    }
    last_byte_ms_ = now_ms;

    if (byte == 0x00U) {
      if (!in_frame_) {
        return FrameStatus::EmptyDelimiter;
      }

      if (overflowed_) {
        ++stats_.overflow_errors;
        reset();
        return FrameStatus::FrameTooLarge;
      }

      if (remaining_in_block_ > 0U) {
        ++stats_.cobs_errors;
        reset();
        return FrameStatus::CobsError;
      }

      const size_t decoded_len = write_idx_;
      reset();

      if (decoded_len < kMinDecodedFrameSize) {
        ++stats_.too_short_errors;
        return FrameStatus::TooShort;
      }

      const size_t payload_and_header_len = decoded_len - 2U;
      const uint16_t expected_crc = Crc16::compute(buf_, payload_and_header_len);
      const uint16_t received_crc =
          static_cast<uint16_t>(buf_[decoded_len - 2U]) |
          (static_cast<uint16_t>(buf_[decoded_len - 1U]) << 8U);

      if (expected_crc != received_crc) {
        ++stats_.crc_errors;
        return FrameStatus::CrcMismatch;
      }

      ++stats_.frames_received;
      if (out_frame != nullptr) {
        out_frame->type = buf_[0];
        out_frame->seq = buf_[1];
        out_frame->body = &buf_[2];
        out_frame->body_len = decoded_len - kMinDecodedFrameSize;
        out_frame->crc16 = received_crc;
      }
      return FrameStatus::FrameReady;
    }

    if (!in_frame_) {
      in_frame_ = true;
      overflowed_ = false;
      write_idx_ = 0U;
      code_ = byte;
      remaining_in_block_ = static_cast<uint8_t>(byte - 1U);
      return FrameStatus::Incomplete;
    }

    if (overflowed_) {
      return FrameStatus::Incomplete;
    }

    if (remaining_in_block_ == 0U) {
      if (code_ < 0xFFU) {
        if (write_idx_ >= rx_max_) {
          overflowed_ = true;
          return FrameStatus::Incomplete;
        }
        buf_[write_idx_++] = 0x00U;
      }
      code_ = byte;
      remaining_in_block_ = static_cast<uint8_t>(byte - 1U);
      return FrameStatus::Incomplete;
    }

    if (write_idx_ >= rx_max_) {
      overflowed_ = true;
      return FrameStatus::Incomplete;
    }
    buf_[write_idx_++] = byte;
    --remaining_in_block_;
    return FrameStatus::Incomplete;
  }

  const FrameStats& stats() const { return stats_; }
  size_t rxMax() const { return rx_max_; }

 private:
  uint8_t* buf_;
  FrameSize rx_max_;
  FrameSize write_idx_;
  uint16_t timeout_ms_;
  uint32_t last_byte_ms_;
  uint8_t code_;
  uint8_t remaining_in_block_;
  bool in_frame_;
  bool overflowed_;
  FrameStats stats_;
};

/**
 * @brief Frame builder and streaming COBS writer.
 *
 * Constructs `type | seq | body[...] | crc16 LE` in `tx_buf[0..tx_max)` and streams
 * COBS-encoded bytes + `0x00` delimiter through any `Sink` implementing `bool writeByte(uint8_t)`
 * (no virtual calls on the hot path).
 */
class FrameWriter {
 public:
  static constexpr size_t kOverhead = 4U;  ///< type(1) + seq(1) + crc16(2)

  FrameWriter() = default;
  FrameWriter(uint8_t* tx_buf, size_t tx_max)
      : tx_buf_(tx_buf), tx_max_(static_cast<FrameSize>(tx_max)) {}

  void init(uint8_t* tx_buf, size_t tx_max) {
    tx_buf_ = tx_buf;
    tx_max_ = static_cast<FrameSize>(tx_max);
  }

  size_t txMax() const { return tx_max_; }
  size_t maxBodySize() const { return (tx_max_ >= kOverhead) ? (tx_max_ - kOverhead) : 0U; }

  /**
   * @brief Begin constructing a frame in `tx_buf_`.
   * @return Pointer to the body buffer (`maxBodySize()` bytes available), or nullptr if `tx_max_ < 4`.
   */
  uint8_t* beginBody() {
    if (tx_max_ < kOverhead || tx_buf_ == nullptr) {
      return nullptr;
    }
    return &tx_buf_[2];
  }

  /**
   * @brief Finalize a frame whose body is already written at `beginBody()[0..body_len)`
   *        and stream-encode it via COBS into `sink`.
   */
  template <typename Sink>
  bool sendPreparedBody(uint8_t type, uint8_t seq, size_t body_len, Sink& sink,
                        size_t peer_rx_max = 0U) {
    const size_t raw_len = body_len + kOverhead;
    if (tx_buf_ == nullptr || raw_len > tx_max_) {
      return false;
    }
    if (peer_rx_max > 0U && raw_len > peer_rx_max) {
      return false;
    }

    tx_buf_[0] = type;
    tx_buf_[1] = seq;
    const uint16_t crc = Crc16::compute(tx_buf_, body_len + 2U);
    tx_buf_[2U + body_len] = static_cast<uint8_t>(crc & 0xFFU);
    tx_buf_[3U + body_len] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);

    return CobsStreamEncoder::encodeFrame(tx_buf_, raw_len, sink);
  }

  /**
   * @brief Copy `body[0..body_len)` into `tx_buf_`, append CRC-16, and stream-encode into `sink`.
   */
  template <typename Sink>
  bool writeFrame(uint8_t type, uint8_t seq, const uint8_t* body, size_t body_len, Sink& sink,
                  size_t peer_rx_max = 0U) {
    if (body_len > maxBodySize()) {
      return false;
    }
    uint8_t* dst = beginBody();
    if (dst == nullptr) {
      return false;
    }
    for (size_t i = 0U; i < body_len; ++i) {
      dst[i] = body[i];
    }
    return sendPreparedBody(type, seq, body_len, sink, peer_rx_max);
  }

 private:
  uint8_t* tx_buf_;
  FrameSize tx_max_;
};

}  // namespace core
}  // namespace rosduino

#endif  // ROSDUINO_CORE_FRAME_H_
