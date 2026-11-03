// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_CORE_CRC16_H_
#define ROSDUINO_CORE_CRC16_H_

#include <stddef.h>
#include <stdint.h>

namespace rosduino {
namespace core {

/**
 * @brief CRC-16/CCITT-FALSE calculator (poly=0x1021, init=0xFFFF, refIn=false, refOut=false, xorOut=0x0000).
 */
class Crc16 {
 public:
  static constexpr uint16_t kInit = 0xFFFFU;
  static constexpr uint16_t kPoly = 0x1021U;

  static inline uint16_t updateByte(uint16_t crc, uint8_t byte) {
    crc ^= static_cast<uint16_t>(byte) << 8U;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1U) ^ kPoly);
      } else {
        crc = static_cast<uint16_t>(crc << 1U);
      }
    }
    return crc;
  }

  static inline uint16_t update(uint16_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      crc = updateByte(crc, data[i]);
    }
    return crc;
  }

  static inline uint16_t compute(const uint8_t* data, size_t len) {
    return update(kInit, data, len);
  }
};

}  // namespace core
}  // namespace rosduino

#endif  // ROSDUINO_CORE_CRC16_H_
