// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_SHIM_AVR_PGMSPACE_H_
#define ROSDUINO_SHIM_AVR_PGMSPACE_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PROGMEM
#define PSTR(s) (s)

inline uint8_t pgm_read_byte(const void* p) { return *static_cast<const uint8_t*>(p); }

inline size_t strlen_P(const char* s) { return s == nullptr ? 0U : strlen(s); }

#endif  // ROSDUINO_SHIM_AVR_PGMSPACE_H_
