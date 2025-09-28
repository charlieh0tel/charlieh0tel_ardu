#pragma once

#include "Arduino.h"

struct EncoderReading {
  long position;
  float rate_rpm;
  bool switch_was_pressed;
};


struct EncoderConfig {
  unsigned int pulses_per_revolution;
  uint8_t pin_a;
  uint8_t pin_mode_a;
  uint8_t pin_b;
  uint8_t pin_mode_b;
  uint8_t pin_switch;
  uint8_t pin_mode_switch;

  using isr_t = void (*)();
  isr_t quadrature_isr;
  isr_t switch_isr;
};

// --- Encoder Class ---
class EncoderState {
public:
  EncoderState() {}

  void begin(const EncoderConfig& config) {
    this->pin_a = config.pin_a;
    this->pin_b = config.pin_b;
    this->pin_switch = config.pin_switch;
    this->pulses_per_revolution = config.pulses_per_revolution;

    pinMode(this->pin_a, config.pin_mode_a);
    pinMode(this->pin_b, config.pin_mode_b);
    pinMode(this->pin_switch, config.pin_mode_switch);

    this->last_switch_interrupt_time = millis();

    attachInterrupt(digitalPinToInterrupt(config.pin_a), config.quadrature_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config.pin_b), config.quadrature_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config.pin_switch), config.switch_isr, CHANGE);

    noInterrupts();
    this->last_rate_calc_time = micros();
    this->last_rate_calc_position = this->position;
    interrupts();
  }

  EncoderReading read() {
    EncoderReading currentReading;
    noInterrupts();

    unsigned long current_time = micros();
    unsigned long time_diff = current_time - this->last_rate_calc_time;

    if (time_diff > kRateMinTimeForUpdateMicros) {
      long position_diff = this->position - this->last_rate_calc_position;

      float pulses_per_second = (float)position_diff * 1000000.0 / time_diff;

      this->rate_rpm = (pulses_per_second / this->pulses_per_revolution) * 60.0;

      this->last_rate_calc_position = this->position;
      this->last_rate_calc_time = current_time;
    }

    currentReading.position = this->position;
    currentReading.rate_rpm = this->rate_rpm;
    currentReading.switch_was_pressed = this->switch_pressed;
    this->switch_pressed = false;
    interrupts();
    return currentReading;
  }

  void processQuadrature() {
    int8_t a = digitalRead(this->pin_a);
    int8_t b = digitalRead(this->pin_b);
    int8_t encoded_state = (a << 1) | b;

    if (encoded_state != this->last_encoded_state) {
      uint8_t history = (this->last_encoded_state << 2) | encoded_state;
      float direction = 0.0;
      switch (history) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
          direction = -1.0;
          break;
        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
          direction = 1.0;
          break;
        default:
          return;
      }

      if (direction != this->last_direction) {
        this->last_direction = direction;
      } else {
        this->position += (long)direction;
      }
      this->last_encoded_state = encoded_state;
    }
  }

  void processSwitch() {
    unsigned long interruptTime = millis();
    if (interruptTime - this->last_switch_interrupt_time > kSwitchDebounceTimeMillis) {
      if (digitalRead(this->pin_switch) == LOW) {
        this->switch_pressed = true;
      }
      this->last_switch_interrupt_time = interruptTime;
    }
  }

  void resetPosition() {
    noInterrupts();
    this->position = 0;
    interrupts();
  }

private:
  static constexpr unsigned long kRateMinTimeForUpdateMicros = 100 * 1000;
  static constexpr unsigned long kSwitchDebounceTimeMillis = 50;

  uint8_t pin_a, pin_b, pin_switch;
  unsigned int pulses_per_revolution;

  volatile long position = 0;
  volatile float rate_rpm = 0.0;
  volatile int8_t last_encoded_state = 0;
  volatile int8_t last_direction = 0;
  volatile long last_rate_calc_position = 0;
  volatile unsigned long last_rate_calc_time = 0;

  volatile bool switch_pressed = false;
  volatile unsigned long last_switch_interrupt_time = 0;
};
