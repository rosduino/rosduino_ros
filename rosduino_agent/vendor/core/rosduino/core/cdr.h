// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_CORE_CDR_H_
#define ROSDUINO_CORE_CDR_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace rosduino {

/**
 * @brief Fixed-capacity sequence container for ROS 2 `T[]` and `T[<=Cap]` fields without heap.
 */
template <typename T, size_t Cap = 8U>
struct Sequence {
  T data[Cap];
  size_t size;
  bool truncated;

  constexpr Sequence() : data(), size(0U), truncated(false) {}

  static constexpr size_t capacity() { return Cap; }

  void clear() {
    size = 0U;
    truncated = false;
  }

  bool push_back(const T& item) {
    if (size >= Cap) {
      truncated = true;
      return false;
    }
    data[size++] = item;
    return true;
  }

  T& operator[](size_t idx) { return data[idx]; }
  const T& operator[](size_t idx) const { return data[idx]; }
};

namespace core {

/**
 * @brief Bit-level IEEE-754 conversion between 32-bit `float` (binary32) and 64-bit `double`
 *        (binary64, stored as `uint64_t` bits). Used on AVR where `sizeof(double) == 4`.
 */
class Float64Converter {
 public:
  static inline uint64_t float32ToFloat64Bits(float value) {
    uint32_t fbits = 0U;
    memcpy(&fbits, &value, sizeof(fbits));

    const uint64_t sign = static_cast<uint64_t>((fbits >> 31U) & 0x01U) << 63U;
    const uint32_t exp32 = (fbits >> 23U) & 0xFFU;
    const uint32_t mant32 = fbits & 0x7FFFFFU;

    if (exp32 == 0xFFU) {
      // Inf or NaN
      const uint64_t exp64 = static_cast<uint64_t>(0x7FFU) << 52U;
      const uint64_t mant64 = static_cast<uint64_t>(mant32) << 29U;
      return sign | exp64 | mant64;
    }

    if (exp32 == 0U) {
      if (mant32 == 0U) {
        return sign;  // +/- 0.0
      }
      // Subnormal float32 -> normalized float64
      uint32_t m = mant32;
      int32_t exp = -126;
      while ((m & 0x800000U) == 0U) {
        m <<= 1U;
        --exp;
      }
      m &= 0x7FFFFFU;
      const uint64_t exp64 = static_cast<uint64_t>(exp + 1023) << 52U;
      const uint64_t mant64 = static_cast<uint64_t>(m) << 29U;
      return sign | exp64 | mant64;
    }

    const uint64_t exp64 = static_cast<uint64_t>(static_cast<int32_t>(exp32) - 127 + 1023) << 52U;
    const uint64_t mant64 = static_cast<uint64_t>(mant32) << 29U;
    return sign | exp64 | mant64;
  }

  static inline float float64BitsToFloat32(uint64_t dbits) {
    const uint32_t sign = static_cast<uint32_t>((dbits >> 63U) & 0x01U) << 31U;
    const int32_t exp64 = static_cast<int32_t>((dbits >> 52U) & 0x7FFU);
    const uint64_t mant64 = dbits & 0x000FFFFFFFFFFFFFULL;

    uint32_t fbits = 0U;
    if (exp64 == 0x7FF) {
      // Inf or NaN
      const uint32_t mant32 =
          (mant64 != 0ULL) ? (static_cast<uint32_t>(mant64 >> 29U) | 0x400000U) : 0U;
      fbits = sign | (0xFFUL << 23U) | mant32;
    } else if (exp64 == 0) {
      // Zero or float64 subnormal (underflows to +/-0.0 in float32)
      fbits = sign;
    } else {
      const int32_t exp32 = exp64 - 1023 + 127;
      if (exp32 >= 0xFF) {
        // Overflow to +/-Inf
        fbits = sign | (0xFFUL << 23U);
      } else if (exp32 <= 0) {
        // Underflow or subnormal in float32
        if (exp32 < -23) {
          fbits = sign;
        } else {
          const uint64_t full_mant = (1ULL << 52U) | mant64;
          const uint32_t shift = static_cast<uint32_t>(29 + (1 - exp32));
          const uint32_t mant32 = static_cast<uint32_t>(full_mant >> shift);
          fbits = sign | mant32;
        }
      } else {
        // Round to nearest even on bit 28
        uint32_t mant32 = static_cast<uint32_t>(mant64 >> 29U);
        const uint32_t round_bit = static_cast<uint32_t>((mant64 >> 28U) & 1ULL);
        const uint64_t sticky_bits = mant64 & ((1ULL << 28U) - 1ULL);
        uint32_t uexp32 = static_cast<uint32_t>(exp32);
        if (round_bit != 0U && (sticky_bits != 0ULL || (mant32 & 1U) != 0U)) {
          ++mant32;
          if (mant32 == 0x800000U) {
            mant32 = 0U;
            ++uexp32;
            if (uexp32 >= 0xFFU) {
              fbits = sign | (0xFFUL << 23U);
              float out = 0.0f;
              memcpy(&out, &fbits, sizeof(out));
              return out;
            }
          }
        }
        fbits = sign | (uexp32 << 23U) | mant32;
      }
    }

    float out = 0.0f;
    memcpy(&out, &fbits, sizeof(out));
    return out;
  }
};

/**
 * @brief XCDR1 Little-Endian serializer.
 *
 * Alignment (`1, 2, 4, 8`) is always calculated relative to `offset_ == 0` (the start of the
 * CDR payload, immediately after the frame's `id`/`req_id` field).
 */
class CdrWriter {
 public:
  CdrWriter(uint8_t* buf, size_t capacity)
      : buf_(buf), capacity_(capacity), offset_(0U), ok_(buf != nullptr || capacity == 0U) {}

  size_t size() const { return offset_; }
  size_t capacity() const { return capacity_; }
  size_t remaining() const { return (offset_ <= capacity_) ? (capacity_ - offset_) : 0U; }
  bool ok() const { return ok_; }

  bool align(size_t alignment) {
    if (!ok_ || alignment <= 1U) {
      return ok_;
    }
    const size_t rem = offset_ % alignment;
    if (rem != 0U) {
      const size_t pad = alignment - rem;
      if (offset_ + pad > capacity_) {
        ok_ = false;
        return false;
      }
      for (size_t i = 0U; i < pad; ++i) {
        buf_[offset_++] = 0x00U;
      }
    }
    return true;
  }

  bool writeUInt8(uint8_t v) {
    if (!ok_ || offset_ + 1U > capacity_) {
      ok_ = false;
      return false;
    }
    buf_[offset_++] = v;
    return true;
  }

  bool writeInt8(int8_t v) { return writeUInt8(static_cast<uint8_t>(v)); }

  bool writeBool(bool v) { return writeUInt8(v ? 1U : 0U); }

  bool writeUInt16(uint16_t v) {
    if (!align(2U) || offset_ + 2U > capacity_) {
      ok_ = false;
      return false;
    }
    buf_[offset_++] = static_cast<uint8_t>(v & 0xFFU);
    buf_[offset_++] = static_cast<uint8_t>((v >> 8U) & 0xFFU);
    return true;
  }

  bool writeInt16(int16_t v) { return writeUInt16(static_cast<uint16_t>(v)); }

  bool writeUInt32(uint32_t v) {
    if (!align(4U) || offset_ + 4U > capacity_) {
      ok_ = false;
      return false;
    }
    buf_[offset_++] = static_cast<uint8_t>(v & 0xFFU);
    buf_[offset_++] = static_cast<uint8_t>((v >> 8U) & 0xFFU);
    buf_[offset_++] = static_cast<uint8_t>((v >> 16U) & 0xFFU);
    buf_[offset_++] = static_cast<uint8_t>((v >> 24U) & 0xFFU);
    return true;
  }

  bool writeInt32(int32_t v) { return writeUInt32(static_cast<uint32_t>(v)); }

  bool writeUInt64(uint64_t v) {
    if (!align(8U) || offset_ + 8U > capacity_) {
      ok_ = false;
      return false;
    }
    for (uint8_t i = 0U; i < 8U; ++i) {
      buf_[offset_++] = static_cast<uint8_t>((v >> (8U * i)) & 0xFFU);
    }
    return true;
  }

  bool writeInt64(int64_t v) { return writeUInt64(static_cast<uint64_t>(v)); }

  bool writeFloat32(float v) {
    uint32_t bits = 0U;
    memcpy(&bits, &v, sizeof(bits));
    return writeUInt32(bits);
  }

  bool writeFloat64FromFloat32(float v) {
    return writeUInt64(Float64Converter::float32ToFloat64Bits(v));
  }

  bool writeFloat64(double v) {
#if defined(__AVR__)
    return writeFloat64FromFloat32(static_cast<float>(v));
#else
    uint64_t bits = 0ULL;
    memcpy(&bits, &v, sizeof(bits));
    return writeUInt64(bits);
#endif
  }

  bool writeString(const char* str) {
    const char* s = (str != nullptr) ? str : "";
    const size_t slen = strlen(s);
    const uint32_t len_with_nul = static_cast<uint32_t>(slen + 1U);
    if (!writeUInt32(len_with_nul)) {
      return false;
    }
    if (offset_ + len_with_nul > capacity_) {
      ok_ = false;
      return false;
    }
    if (slen > 0U) {
      memcpy(&buf_[offset_], s, slen);
      offset_ += slen;
    }
    buf_[offset_++] = 0x00U;
    return true;
  }

  template <typename T, typename WriteElemFn>
  bool writeArray(const T* arr, size_t count, WriteElemFn write_elem) {
    for (size_t i = 0U; i < count; ++i) {
      if (!write_elem(*this, arr[i])) {
        return false;
      }
    }
    return ok_;
  }

  template <typename T, size_t Cap, typename WriteElemFn>
  bool writeSequence(const Sequence<T, Cap>& seq, WriteElemFn write_elem) {
    const size_t n = (seq.size <= Cap) ? seq.size : Cap;
    if (!writeUInt32(static_cast<uint32_t>(n))) {
      return false;
    }
    return writeArray(seq.data, n, write_elem);
  }

 private:
  uint8_t* buf_;
  size_t capacity_;
  size_t offset_;
  bool ok_;
};

/**
 * @brief XCDR1 Little-Endian deserializer.
 *
 * - Alignment is relative to `offset_ == 0` (start of CDR payload).
 * - Alignment padding bytes between fields are not value-checked.
 * - Trailing padding bytes (`0..3` bytes from Fast-DDS) after all fields are read are ignored.
 * - Incoming strings are zero-copy pointers into `buf_`.
 */
class CdrReader {
 public:
  CdrReader(const uint8_t* buf, size_t len)
      : buf_(buf), len_(len), offset_(0U), ok_(buf != nullptr || len == 0U) {}

  size_t offset() const { return offset_; }
  size_t size() const { return len_; }
  size_t remaining() const { return (offset_ <= len_) ? (len_ - offset_) : 0U; }
  bool ok() const { return ok_; }

  bool align(size_t alignment) {
    if (!ok_ || alignment <= 1U) {
      return ok_;
    }
    const size_t rem = offset_ % alignment;
    if (rem != 0U) {
      const size_t pad = alignment - rem;
      if (offset_ + pad > len_) {
        ok_ = false;
        return false;
      }
      offset_ += pad;
    }
    return true;
  }

  bool readUInt8(uint8_t* out) {
    if (!ok_ || out == nullptr || offset_ + 1U > len_) {
      ok_ = false;
      return false;
    }
    *out = buf_[offset_++];
    return true;
  }

  bool readInt8(int8_t* out) {
    uint8_t u = 0U;
    if (!readUInt8(&u) || out == nullptr) {
      ok_ = false;
      return false;
    }
    *out = static_cast<int8_t>(u);
    return true;
  }

  bool readBool(bool* out) {
    uint8_t u = 0U;
    if (!readUInt8(&u) || out == nullptr) {
      ok_ = false;
      return false;
    }
    *out = (u != 0U);
    return true;
  }

  bool readUInt16(uint16_t* out) {
    if (!align(2U) || out == nullptr || offset_ + 2U > len_) {
      ok_ = false;
      return false;
    }
    *out = static_cast<uint16_t>(buf_[offset_]) |
           (static_cast<uint16_t>(buf_[offset_ + 1U]) << 8U);
    offset_ += 2U;
    return true;
  }

  bool readInt16(int16_t* out) {
    uint16_t u = 0U;
    if (!readUInt16(&u) || out == nullptr) {
      ok_ = false;
      return false;
    }
    *out = static_cast<int16_t>(u);
    return true;
  }

  bool readUInt32(uint32_t* out) {
    if (!align(4U) || out == nullptr || offset_ + 4U > len_) {
      ok_ = false;
      return false;
    }
    *out = static_cast<uint32_t>(buf_[offset_]) |
           (static_cast<uint32_t>(buf_[offset_ + 1U]) << 8U) |
           (static_cast<uint32_t>(buf_[offset_ + 2U]) << 16U) |
           (static_cast<uint32_t>(buf_[offset_ + 3U]) << 24U);
    offset_ += 4U;
    return true;
  }

  bool readInt32(int32_t* out) {
    uint32_t u = 0U;
    if (!readUInt32(&u) || out == nullptr) {
      ok_ = false;
      return false;
    }
    *out = static_cast<int32_t>(u);
    return true;
  }

  bool readUInt64(uint64_t* out) {
    if (!align(8U) || out == nullptr || offset_ + 8U > len_) {
      ok_ = false;
      return false;
    }
    uint64_t val = 0ULL;
    for (uint8_t i = 0U; i < 8U; ++i) {
      val |= static_cast<uint64_t>(buf_[offset_ + i]) << (8U * i);
    }
    offset_ += 8U;
    *out = val;
    return true;
  }

  bool readInt64(int64_t* out) {
    uint64_t u = 0ULL;
    if (!readUInt64(&u) || out == nullptr) {
      ok_ = false;
      return false;
    }
    *out = static_cast<int64_t>(u);
    return true;
  }

  bool readFloat32(float* out) {
    uint32_t bits = 0U;
    if (!readUInt32(&bits) || out == nullptr) {
      ok_ = false;
      return false;
    }
    memcpy(out, &bits, sizeof(*out));
    return true;
  }

  bool readFloat64AsFloat32(float* out) {
    uint64_t bits = 0ULL;
    if (!readUInt64(&bits) || out == nullptr) {
      ok_ = false;
      return false;
    }
    *out = Float64Converter::float64BitsToFloat32(bits);
    return true;
  }

  bool readFloat64(double* out) {
    if (out == nullptr) {
      ok_ = false;
      return false;
    }
#if defined(__AVR__)
    float f = 0.0f;
    if (!readFloat64AsFloat32(&f)) {
      return false;
    }
    *out = static_cast<double>(f);
    return true;
#else
    uint64_t bits = 0ULL;
    if (!readUInt64(&bits)) {
      return false;
    }
    memcpy(out, &bits, sizeof(*out));
    return true;
#endif
  }

  bool readString(const char** out_str, uint32_t* out_len = nullptr) {
    if (out_str == nullptr) {
      ok_ = false;
      return false;
    }
    uint32_t len_with_nul = 0U;
    if (!readUInt32(&len_with_nul) || len_with_nul == 0U || offset_ + len_with_nul > len_) {
      ok_ = false;
      return false;
    }
    if (buf_[offset_ + len_with_nul - 1U] != 0x00U) {
      ok_ = false;
      return false;
    }
    *out_str = reinterpret_cast<const char*>(&buf_[offset_]);
    if (out_len != nullptr) {
      *out_len = len_with_nul - 1U;
    }
    offset_ += len_with_nul;
    return true;
  }

  template <typename T, typename ReadElemFn>
  bool readArray(T* arr, size_t count, ReadElemFn read_elem) {
    for (size_t i = 0U; i < count; ++i) {
      if (!read_elem(*this, &arr[i])) {
        return false;
      }
    }
    return ok_;
  }

  template <typename T, size_t Cap, typename ReadElemFn>
  bool readSequence(Sequence<T, Cap>* seq, ReadElemFn read_elem) {
    if (seq == nullptr) {
      ok_ = false;
      return false;
    }
    seq->clear();
    uint32_t count = 0U;
    if (!readUInt32(&count)) {
      return false;
    }
    for (uint32_t i = 0U; i < count; ++i) {
      T elem;
      memset(&elem, 0, sizeof(elem));
      if (!read_elem(*this, &elem)) {
        return false;
      }
      seq->push_back(elem);
    }
    return ok_;
  }

 private:
  const uint8_t* buf_;
  size_t len_;
  size_t offset_;
  bool ok_;
};

}  // namespace core
}  // namespace rosduino

#endif  // ROSDUINO_CORE_CDR_H_
