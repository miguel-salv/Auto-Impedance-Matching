/*
 * impedance_matcher.ino
 * Impedance Matcher — Teensy 4.1
 *
 * Dependencies (Arduino Library Manager):
 *   Adafruit SSD1306, Adafruit GFX, TMCStepper
 */

#include <Arduino.h>
#include "include/pins_config.h"
#include "include/app_state.h"
#include "include/matching.h"
#include "include/encoder.h"
#include "include/ui.h"

// ─── Global state (definitions; declarations in include/app_state.h) ────────
AppState state   = S_HOME;
OpMode   opMode  = MODE_AUTO;
bool     radioTX = false;

long  motor1Pos = 0;
long  motor2Pos = 0;
float motor1_pos = 0.0f;
float motor2_pos = 0.0f;
float dM1 = 0.1f;
float dM2 = 0.1f;

bool  atMatch   = false;
float lastVSWR  = 1.0f;
float lastFwdV  = 0.0f;
float lastRevV  = 0.0f;

void setup() {
  analogReadResolution(12);
  Serial.begin(500000);
  while (!Serial && millis() < 3000)
    ;

  matching_init_motor_pins();
  matching_init_uart();
  encoder_init();

  if (!ui_init_display()) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  matching_homing();
}

void loop() {
  matching_tick();

  int  delta   = 0;
  bool pressed = false;
  encoder_poll(&delta, &pressed);

  const bool inAutoHome = (opMode == MODE_AUTO && state == S_HOME);
  const bool userInput  = (delta != 0) || pressed;
  const bool doDraw     = ui_should_redraw(userInput, inAutoHome);

  ui_tick(delta, pressed, doDraw);

  if (inAutoHome && !doDraw) {
    yield();
  } else {
    delay(SCHED_UI_FRAME_MS);
  }
}
