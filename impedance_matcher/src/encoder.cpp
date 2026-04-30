#include "../include/encoder.h"
#include "../include/pins_config.h"

#include <Arduino.h>

// ─── Quadrature: ISR on A and B (same handler). Table gives ±1 per valid Gray step. ───
static volatile int   encRaw    = 0;
static volatile uint8_t encLastAB = 0;

static const int8_t kQuadTable[16] = {
    0, +1, -1, 0,
    -1, 0, 0, +1,
    +1, 0, 0, -1,
    0, -1, +1, 0,
};

static void encoderIsr() {
  uint8_t a    = (uint8_t)digitalRead(ENC_A);
  uint8_t b    = (uint8_t)digitalRead(ENC_B);
  uint8_t newAB = (uint8_t)((a << 1) | b);
  uint8_t idx   = (uint8_t)((encLastAB << 2) | newAB);
  int8_t step   = kQuadTable[idx];
  if (step != 0) {
    encRaw += step;
    encLastAB = newAB;
  }
}

// Typical mechanical detent = 4 quadrature edges per click (full A/B tracking).
#define ENC_COUNTS_PER_CLICK 4

static int encAccum = 0;

// ─── Button: FALLING counted in ISR; loop only debounces with millis (no delays). ───
static volatile uint32_t btnFallEvents = 0;

static void btnFallIsr() {
  btnFallEvents++;
}

#define DEBOUNCE_MS 40
static uint32_t lastBtnAcceptMs = 0;

static int consumeDelta() {
  noInterrupts();
  int d = encRaw;
  encRaw = 0;
  interrupts();
  encAccum += d;
  int clicks = encAccum / ENC_COUNTS_PER_CLICK;
  encAccum -= clicks * ENC_COUNTS_PER_CLICK;
  return clicks * SCROLL_DIR;
}

static bool consumeButton() {
  uint32_t falls;
  noInterrupts();
  falls         = btnFallEvents;
  btnFallEvents = 0;
  interrupts();

  if (falls == 0) {
    return false;
  }

  uint32_t now = millis();
  if ((now - lastBtnAcceptMs) <= DEBOUNCE_MS) {
    noInterrupts();
    btnFallEvents += falls;
    interrupts();
    return false;
  }

  if (digitalRead(ENC_BTN) != LOW) {
    return false;
  }

  lastBtnAcceptMs = now;
  return true;
}

void encoder_init() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);

  uint8_t a = (uint8_t)digitalRead(ENC_A);
  uint8_t b = (uint8_t)digitalRead(ENC_B);
  encLastAB = (uint8_t)((a << 1) | b);

  int pinA = digitalPinToInterrupt(ENC_A);
  int pinB = digitalPinToInterrupt(ENC_B);
  int pinT = digitalPinToInterrupt(ENC_BTN);
  attachInterrupt(pinA, encoderIsr, CHANGE);
  attachInterrupt(pinB, encoderIsr, CHANGE);
  attachInterrupt(pinT, btnFallIsr, FALLING);
}

void encoder_poll(int* outDelta, bool* outPressed) {
  *outDelta   = consumeDelta();
  *outPressed = consumeButton();
}
