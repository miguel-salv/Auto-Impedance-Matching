#pragma once

enum AppState { S_HOME, S_MENU, S_MOTOR1, S_MOTOR2, S_METRICS };
enum OpMode   { MODE_AUTO, MODE_MANUAL };

// Plain int in APIs to avoid Arduino enum forward-declaration issues.
enum MenuID {
  M_MODE        = 0,
  M_MOTOR1      = 1,
  M_MOTOR2      = 2,
  M_ADV_METRICS = 3,
  M_BACK        = 4,
  M_COUNT       = 5
};

extern AppState state;
extern OpMode   opMode;
extern bool     radioTX;

extern long  motor1Pos;
extern long  motor2Pos;
extern float motor1_pos;
extern float motor2_pos;
extern float dM1;
extern float dM2;

extern bool  atMatch;
extern float lastVSWR;
extern float lastFwdV;
extern float lastRevV;
