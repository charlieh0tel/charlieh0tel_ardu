#include <Arduino.h>

#include "Encoder.h"

EncoderState encoder;

void quadrature_isr() {
  encoder.processQuadrature();
}

void switch_isr() {
  encoder.processSwitch();
}

void setup() {
  Serial.begin(115200);

  EncoderConfig config = {
    .pulses_per_revolution = 128,
    .pin_a = 5, .pin_mode_a = INPUT, 
    .pin_b = 6, .pin_mode_b = INPUT,
    .pin_switch = 12, .pin_mode_switch = INPUT_PULLUP,

    .quadrature_isr = quadrature_isr,
    .switch_isr = switch_isr
  };
  encoder.begin(config);
}

void loop() {
  EncoderReading reading = encoder.read();

  Serial.print("Position: ");
  Serial.print(reading.position);
  Serial.print(" | Rate: ");
  Serial.print(reading.rate_rpm);
  Serial.println(" rpm");

  if (reading.switch_was_pressed) {
    Serial.println("--> switch pressed");
    encoder.resetPosition();
  }

  delay(100);
}