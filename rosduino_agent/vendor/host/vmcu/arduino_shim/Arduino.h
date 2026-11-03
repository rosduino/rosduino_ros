// SPDX-License-Identifier: Apache-2.0
#ifndef ROSDUINO_SHIM_ARDUINO_H_
#define ROSDUINO_SHIM_ARDUINO_H_

#include <stddef.h>
#include <stdint.h>

#include "avr/pgmspace.h"

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define LED_BUILTIN 13
#define A0 14
#define A1 15

class __FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper*>(string_literal))

uint32_t millis();
uint32_t micros();
void delay(unsigned long ms);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);

class HardwareSerial {
 public:
  static constexpr uint8_t kRxFifo = 64U;

  HardwareSerial();

  void begin(unsigned long baud);
  int available();
  int read();
  size_t write(uint8_t b);

  void attach(int fd);
  void close();
  bool connected() const;
  int nativeFd() const;
  unsigned long uartBaud() const { return baud_; }
  uint32_t uartOverruns() const { return overruns_; }

 private:
  void pumpRx();
  void waitTxSlot();
  uint32_t byteTimeUs() const;
  bool fifoPush(uint8_t b);
  int fifoPop();

  int fd_;
  unsigned long baud_;
  uint32_t tx_next_us_;
  uint32_t rx_last_us_;
  uint32_t rx_credit_us_;
  uint32_t overruns_;
  uint8_t rx_fifo_[kRxFifo];
  uint8_t rx_head_;
  uint8_t rx_tail_;
  uint8_t rx_count_;
};

using Stream = HardwareSerial;

extern HardwareSerial Serial;

#endif  // ROSDUINO_SHIM_ARDUINO_H_
