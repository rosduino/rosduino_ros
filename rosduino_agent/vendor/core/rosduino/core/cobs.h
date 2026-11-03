// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_CORE_COBS_H_
#define ROSDUINO_CORE_COBS_H_

#include <stddef.h>
#include <stdint.h>

namespace rosduino {
namespace core {

/**
 * @brief Consistent Overhead Byte Stuffing (COBS) encoder and in-place decoder.
 *
 * Implements:
 * - In-place decoding in a single buffer without heap or extra scratch space.
 * - Streaming encoding (pushing chunks of bytes into a Sink with `bool writeByte(uint8_t)`)
 *   using a 254-byte block accumulator or direct buffer-to-buffer encoding.
 */
class Cobs {
 public:
  /**
   * @brief Compute the maximum COBS-encoded length for a raw payload of size `raw_len`
   *        (excluding the trailing 0x00 frame delimiter).
   */
  static constexpr size_t maxEncodedSize(size_t raw_len) {
    return raw_len + (raw_len / 254U) + 1U;
  }

  /**
   * @brief Encode `raw[0..raw_len)` into `out[0..out_cap)` (without trailing 0x00 delimiter).
   * @return Number of encoded bytes written, or 0 if `out_cap` is too small.
   */
  static inline size_t encode(const uint8_t* raw, size_t raw_len, uint8_t* out, size_t out_cap) {
    if (out_cap == 0U) {
      return 0U;
    }
    size_t code_idx = 0U;
    size_t write_idx = 1U;
    uint8_t code = 1U;

    for (size_t i = 0; i < raw_len; ++i) {
      const uint8_t b = raw[i];
      if (b == 0U) {
        out[code_idx] = code;
        code_idx = write_idx;
        if (write_idx >= out_cap) {
          return 0U;
        }
        ++write_idx;
        code = 1U;
      } else {
        if (write_idx >= out_cap) {
          return 0U;
        }
        out[write_idx++] = b;
        ++code;
        if (code == 0xFFU) {
          out[code_idx] = code;
          code_idx = write_idx;
          if (write_idx >= out_cap) {
            return 0U;
          }
          ++write_idx;
          code = 1U;
        }
      }
    }

    out[code_idx] = code;
    return write_idx;
  }

  /**
   * @brief Decode COBS-encoded bytes in-place inside `buf[0..encoded_len)`.
   *        `buf` must NOT include the trailing 0x00 frame delimiter; any 0x00 byte inside
   *        `buf[0..encoded_len)` is a framing error.
   *
   * @param buf         Buffer containing COBS-encoded frame; overwritten with decoded bytes.
   * @param encoded_len Length of the COBS-encoded data (before the 0x00 delimiter).
   * @param decoded_len Output: number of decoded bytes written into `buf`.
   * @return true if valid COBS data was decoded, false if malformed/truncated.
   */
  static inline bool decodeInPlace(uint8_t* buf, size_t encoded_len, size_t* decoded_len) {
    if (decoded_len == nullptr) {
      return false;
    }
    *decoded_len = 0U;
    if (encoded_len == 0U) {
      return true;
    }

    size_t read_idx = 0U;
    size_t write_idx = 0U;

    while (read_idx < encoded_len) {
      const uint8_t code = buf[read_idx++];
      if (code == 0U) {
        return false;
      }

      const uint8_t copy_count = static_cast<uint8_t>(code - 1U);
      if (read_idx + copy_count > encoded_len) {
        return false;
      }

      for (uint8_t i = 0U; i < copy_count; ++i) {
        const uint8_t b = buf[read_idx++];
        if (b == 0U) {
          return false;
        }
        buf[write_idx++] = b;
      }

      if (code < 0xFFU && read_idx < encoded_len) {
        buf[write_idx++] = 0U;
      }
    }

    *decoded_len = write_idx;
    return true;
  }
};

/**
 * @brief Streaming COBS encoder over a pre-constructed raw frame buffer or byte stream.
 *
 * Scans ahead up to 254 bytes to emit the COBS code byte followed by non-zero data bytes
 * directly into `Sink::writeByte(uint8_t)` without needing an extra TX buffer.
 */
class CobsStreamEncoder {
 public:
  /**
   * @brief Stream-encode `raw[0..raw_len)` followed by the 0x00 delimiter into `sink`.
   * @tparam Sink Type providing `bool writeByte(uint8_t)`.
   */
  template <typename Sink>
  static inline bool encodeFrame(const uint8_t* raw, size_t raw_len, Sink& sink) {
    size_t idx = 0U;
    for (;;) {
      uint8_t code = 1U;
      size_t scan = idx;
      while (scan < raw_len && raw[scan] != 0U && code < 0xFFU) {
        ++scan;
        ++code;
      }

      if (!sink.writeByte(code)) {
        return false;
      }
      for (size_t i = idx; i < scan; ++i) {
        if (!sink.writeByte(raw[i])) {
          return false;
        }
      }

      idx = scan;
      if (idx == raw_len) {
        if (code == 0xFFU) {
          if (!sink.writeByte(0x01U)) {
            return false;
          }
        }
        break;
      }

      if (code < 0xFFU) {
        // Consume the zero byte that ended this block
        ++idx;
        if (idx == raw_len) {
          // Trailing zero at the end of raw payload requires a final 0x01 code byte
          if (!sink.writeByte(0x01U)) {
            return false;
          }
          break;
        }
      }
    }

    return sink.writeByte(0x00U);
  }
};

}  // namespace core
}  // namespace rosduino

#endif  // ROSDUINO_CORE_COBS_H_
