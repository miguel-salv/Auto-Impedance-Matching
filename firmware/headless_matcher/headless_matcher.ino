/*
 * headless_matcher.ino
 * Headless Matcher — Teensy 4.1
 *
 * Dependencies (Arduino Library Manager):
 *   TMCStepper
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

const int STEP_PIN_1 = 2;
const int DIR_PIN_1 = 3;
const int STEP_PIN_2 = 9;
const int DIR_PIN_2 = 10;
const int FWD_PIN = 25;
const int REV_PIN = 24;
const int TRANSMIT_PIN = 31;

#define SERIAL_PORT Serial1
#define R_SENSE 0.11f
#define DRV_ADDRESS_1 0b00
#define DRV_ADDRESS_2 0b01

TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRV_ADDRESS_1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRV_ADDRESS_2);

#define STEP_DELAY 200
#define STEPS_PER_RAD 63.66197f
#define MICROSTEPS_PER_STEP 32
#define MAX_ROT 3.14159f
#define GRAD_SCALE 0.025f
#define MAX_STEPSIZE (PI / 36.0f)
#define MIN_STEPSIZE (PI / 700.0f)
#define SCHED_LOOP_MS 16u

const float samp_num = 300.0f;

enum OpMode { MODE_AUTO, MODE_MANUAL };
OpMode opMode = MODE_AUTO;
bool radioTX = false;
bool atMatch = false;

float motor1_pos = 2.0f * PI;
float motor2_pos = 2.0f * PI;
float dM1 = 0.1f;
float dM2 = 0.1f;

long motor1Pos = 0;
long motor2Pos = 0;

float lastVSWR = 1.0f;
float lastFwdV = 0.0f;
float lastRevV = 0.0f;
static uint32_t lastStatusMs = 0;
bool csvStreamEnabled = false;

static float analogReadMilliVolts(int pin);
static float turnByRad(TMC2209Stepper &driver, int stepPin, int dirPin, float rads, float &motor_pos, bool ignoreLimits = false);
static void calcGradAndStep(TMC2209Stepper &driver, int stepPin, int dirPin, float &gradient, float &motor_pos);
static float clampMagnitude(float value, float minMagnitude, float maxMagnitude);
static float stepsToRad(int steps);
static int radToSteps(float rads);
static float sampVSWR(int fwd, int rev);
static void takeStep(int stepPin);
static int round_up(float value);
static void printHelp();
static void printStatus();
static void processSerialCommand();
static void handleCommand(char *line);
static void setRadioTX(bool enabled);
static void syncMotorDeg();
static bool parseLongArg(const char *arg, long *out);
static void setMotor1Step(long posDeg);
static void setMotor2Step(long posDeg);

void setup() {
  analogReadResolution(12);

  Serial.begin(500000);
  while (!Serial && millis() < 3000) {
  }

  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(TRANSMIT_PIN, OUTPUT);
  digitalWrite(TRANSMIT_PIN, LOW);  // HIGH = TX on for this hardware; LOW at boot = TX off

  SERIAL_PORT.begin(500000, SERIAL_8N1);
  delay(500);

  Serial.println("Starting Homing Sequence...");
  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, -PI, motor1_pos, true);
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, -PI, motor2_pos, true);
  motor1_pos = 0.0f;
  motor2_pos = 0.0f;
  Serial.println("Finished Homing Sequence...");
  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, PI / 2.0f, motor1_pos, true);
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, PI / 2.0f, motor2_pos, true);

  syncMotorDeg();
  printHelp();
  printStatus();
}

void loop() {
  processSerialCommand();

  if (opMode == MODE_AUTO) {
    if (!atMatch) {
      calcGradAndStep(driver1, STEP_PIN_1, DIR_PIN_1, dM1, motor1_pos);
      calcGradAndStep(driver2, STEP_PIN_2, DIR_PIN_2, dM2, motor2_pos);
    } else {
      sampVSWR(FWD_PIN, REV_PIN);
    }
  } else {
    // Keep telemetry fresh in manual mode too.
    sampVSWR(FWD_PIN, REV_PIN);
  }

  syncMotorDeg();

  const bool statusPeriodically = !csvStreamEnabled;
  if (statusPeriodically && millis() - lastStatusMs >= 1000u) {
    lastStatusMs = millis();
    printStatus();
  }

  delay(SCHED_LOOP_MS);
}

static void processSerialCommand() {
  static char line[64];
  static size_t len = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (len > 0) {
        line[len] = '\0';
        handleCommand(line);
        len = 0;
      }
      continue;
    }
    if (len + 1 < sizeof(line)) {
      line[len++] = c;
    }
  }
}

static void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;

  if (strcasecmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    printHelp();
    return;
  }
  if (strcasecmp(cmd, "mode") == 0) {
    char *arg = strtok(nullptr, " \t");
    if (!arg) {
      Serial.println("ERR mode needs: auto|manual");
      return;
    }
    if (strcasecmp(arg, "auto") == 0) {
      opMode = MODE_AUTO;
      Serial.println("OK mode=auto");
      return;
    }
    if (strcasecmp(arg, "manual") == 0) {
      opMode = MODE_MANUAL;
      Serial.println("OK mode=manual");
      return;
    }
    Serial.println("ERR mode must be auto|manual");
    return;
  }
  if (strcasecmp(cmd, "tx") == 0) {
    char *arg = strtok(nullptr, " \t");
    if (!arg) {
      Serial.println("ERR tx needs: on|off");
      return;
    }
    if (strcasecmp(arg, "on") == 0) {
      setRadioTX(true);
      Serial.println("OK tx=on");
      return;
    }
    if (strcasecmp(arg, "off") == 0) {
      setRadioTX(false);
      Serial.println("OK tx=off");
      return;
    }
    Serial.println("ERR tx must be on|off");
    return;
  }
  if (strcasecmp(cmd, "m1") == 0 || strcasecmp(cmd, "m2") == 0) {
    if (opMode != MODE_MANUAL) {
      Serial.println("ERR manual mode required for motor set");
      return;
    }
    char *arg = strtok(nullptr, " \t");
    long deg = 0;
    if (!parseLongArg(arg, &deg)) {
      Serial.println("ERR motor command needs angle 0..180");
      return;
    }
    deg = constrain(deg, 0L, 180L);
    if (strcasecmp(cmd, "m1") == 0) setMotor1Step(deg);
    else setMotor2Step(deg);
    syncMotorDeg();
    Serial.println("OK motor updated");
    return;
  }
  if (strcasecmp(cmd, "home") == 0) {
    Serial.println("Re-homing...");
    turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, -PI, motor1_pos, true);
    turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, -PI, motor2_pos, true);
    motor1_pos = 0.0f;
    motor2_pos = 0.0f;
    turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, PI / 2.0f, motor1_pos, true);
    turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, PI / 2.0f, motor2_pos, true);
    syncMotorDeg();
    Serial.println("OK homed");
    return;
  }
  if (strcasecmp(cmd, "stream") == 0) {
    char *arg = strtok(nullptr, " \t");
    if (!arg) {
      Serial.println("ERR stream needs: on|off");
      return;
    }
    if (strcasecmp(arg, "on") == 0) {
      csvStreamEnabled = true;
      Serial.println("OK stream=on");
      return;
    }
    if (strcasecmp(arg, "off") == 0) {
      csvStreamEnabled = false;
      Serial.println("OK stream=off");
      return;
    }
    Serial.println("ERR stream must be on|off");
    return;
  }
  Serial.println("ERR unknown command. Type: help");
}

static void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  help              - show commands");
  Serial.println("  mode auto|manual  - set operation mode");
  Serial.println("  tx on|off         - radio transmit switch");
  Serial.println("  m1 <deg>          - set motor 1 (manual only)");
  Serial.println("  m2 <deg>          - set motor 2 (manual only)");
  Serial.println("  home              - run homing sequence");
  Serial.println("  stream on|off     - VSWR_CSV lines for live.py / plot logging");
  Serial.println();
}

static void printStatus() {
  Serial.print("STATE mode=");
  Serial.print(opMode == MODE_AUTO ? "AUTO" : "MANUAL");
  Serial.print(" tx=");
  Serial.print(radioTX ? "ON" : "OFF");
  Serial.print(" atMatch=");
  Serial.print(atMatch ? "1" : "0");
  Serial.print(" m1_deg=");
  Serial.print(motor1Pos);
  Serial.print(" m2_deg=");
  Serial.print(motor2Pos);
  Serial.print(" vswr=");
  Serial.print(lastVSWR, 3);
  Serial.print(" fwdV=");
  Serial.print(lastFwdV, 3);
  Serial.print(" revV=");
  Serial.println(lastRevV, 3);
}

static void syncMotorDeg() {
  motor1Pos = constrain((long)(motor1_pos * (180.0f / PI)), 0L, 180L);
  motor2Pos = constrain((long)(motor2_pos * (180.0f / PI)), 0L, 180L);
}

static void setRadioTX(bool enabled) {
  radioTX = enabled;
  digitalWrite(TRANSMIT_PIN, enabled ? HIGH : LOW);
}

static bool parseLongArg(const char *arg, long *out) {
  if (!arg || !out) return false;
  char *end = nullptr;
  long parsed = strtol(arg, &end, 10);
  if (!end || *end != '\0') return false;
  *out = parsed;
  return true;
}

static void setMotor1Step(long posDeg) {
  float targetRad = posDeg * (PI / 180.0f);
  float delta = targetRad - motor1_pos;
  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, delta, motor1_pos);
}

static void setMotor2Step(long posDeg) {
  float targetRad = posDeg * (PI / 180.0f);
  float delta = targetRad - motor2_pos;
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, delta, motor2_pos);
}

static float analogReadMilliVolts(int pin) {
  int rawValue = analogRead(pin);
  return (rawValue / 4095.0f) * 3300.0f;
}

static float turnByRad(TMC2209Stepper &driver, int stepPin, int dirPin, float rads, float &motor_pos, bool ignoreLimits) {
  (void)driver;
  if (rads == 0.0f) {
    return 0.0f;
  }

  bool isNegativeDir = (rads < 0.0f);
  digitalWrite(dirPin, isNegativeDir);

  float abs_rads = isNegativeDir ? -rads : rads;
  int total_steps = radToSteps(abs_rads);

  float step_increment = stepsToRad(1);
  float position_change = isNegativeDir ? -step_increment : step_increment;

  int steps_taken = 0;
  for (int i = 0; i < total_steps; i++) {
    if (!ignoreLimits) {
      bool hittingUpperLimit = (motor_pos >= MAX_ROT && position_change > 0.0f);
      bool hittingLowerLimit = (motor_pos <= 0.0f && position_change < 0.0f);
      if (hittingUpperLimit || hittingLowerLimit) {
        Serial.println("MOTOR LIMIT REACHED");
        break;
      }
    }

    takeStep(stepPin);
    motor_pos += position_change;
    steps_taken++;
  }

  return steps_taken * position_change;
}

static void calcGradAndStep(TMC2209Stepper &driver, int stepPin, int dirPin, float &gradient, float &motor_pos) {
  float initialCost = sampVSWR(FWD_PIN, REV_PIN);
  float commandedStepSize = gradient * GRAD_SCALE;
  commandedStepSize = clampMagnitude(commandedStepSize, MIN_STEPSIZE, MAX_STEPSIZE);

  bool hittingUpperLimit = (motor_pos >= MAX_ROT - MIN_STEPSIZE);
  bool hittingLowerLimit = (motor_pos <= MIN_STEPSIZE);
  if (hittingUpperLimit) commandedStepSize = -MIN_STEPSIZE;
  if (hittingLowerLimit) commandedStepSize = MIN_STEPSIZE;

  float actualTravel = turnByRad(driver, stepPin, dirPin, commandedStepSize, motor_pos);
  delay(5);
  if (actualTravel != 0.0f) {
    gradient = (initialCost - sampVSWR(FWD_PIN, REV_PIN)) / actualTravel;
  }
}

static float clampMagnitude(float value, float minMagnitude, float maxMagnitude) {
  float magnitude = (value < 0.0f) ? -value : value;
  float sign = (value < 0.0f) ? -1.0f : 1.0f;
  if (magnitude > maxMagnitude) return maxMagnitude * sign;
  if (magnitude < minMagnitude) return minMagnitude * sign;
  return value;
}

static float stepsToRad(int steps) {
  return steps / (STEPS_PER_RAD * MICROSTEPS_PER_STEP);
}

static int radToSteps(float rads) {
  return round_up(rads * (STEPS_PER_RAD * MICROSTEPS_PER_STEP));
}

static float sampVSWR(int fwd, int rev) {
  float sum_fwd = 0.0f;
  float sum_rev = 0.0f;
  for (int i = 0; i < (int)samp_num; i++) {
    sum_fwd += analogReadMilliVolts(fwd);
    sum_rev += analogReadMilliVolts(rev);
    delayMicroseconds(15);
  }

  float averageMv_fwd = sum_fwd / samp_num;
  float averageMv_rev = sum_rev / samp_num;
  float denom = averageMv_fwd - averageMv_rev;
  float vswr = (denom != 0.0f) ? (averageMv_fwd + averageMv_rev) / denom : 99.0f;
  float loss = (vswr - 1.0f);
  loss = loss * loss;

  lastVSWR = vswr;
  lastFwdV = averageMv_fwd / 1000.0f;
  lastRevV = averageMv_rev / 1000.0f;

  if (vswr > 1.4f) atMatch = false;
  if (vswr < 1.2f) atMatch = true;

  if (csvStreamEnabled) {
    Serial.print("VSWR_CSV,");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(vswr, 6);
    Serial.print(",");
    Serial.print(lastFwdV, 6);
    Serial.print(",");
    Serial.print(lastRevV, 6);
    Serial.print(",");
    Serial.print(motor1_pos, 6);
    Serial.print(",");
    Serial.print(motor2_pos, 6);
    Serial.print(",");
    Serial.println(atMatch ? 1 : 0);
  }

  return loss;
}

static void takeStep(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY / 2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY / 2);
}

static int round_up(float value) {
  int truncated = static_cast<int>(value);
  if (value > truncated) {
    return truncated + 1;
  }
  return truncated;
}