#pragma once

// Encoder
#define ENC_A      38
#define ENC_B      39
#define ENC_BTN    40
#define SCROLL_DIR 1  // +1 normal, -1 reversed

// Stepper / RF sense
#define STEP_PIN_1   2
#define DIR_PIN_1    3
#define STEP_PIN_2   9
#define DIR_PIN_2    10
#define FWD_PIN      25
#define REV_PIN      24
#define TRANSMIT_PIN 31

// TMC2209 UART
#define MATCHING_SERIAL_PORT Serial1

#define SCREEN_W 128
#define SCREEN_H 64

#define MOTOR_MIN_POS 0
#define MOTOR_MAX_POS 180

#define DISPLAY_THROTTLE_AUTO_HOME_MS 300u
#define SCHED_UI_FRAME_MS             16u
