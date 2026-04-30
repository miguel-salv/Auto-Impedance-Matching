#include "../include/matching.h"
#include "../include/pins_config.h"
#include "../include/app_state.h"

#include <TMCStepper.h>

#define R_SENSE       0.11f
#define DRV_ADDRESS_1 0b00
#define DRV_ADDRESS_2 0b01

static TMC2209Stepper driver1(&MATCHING_SERIAL_PORT, R_SENSE, DRV_ADDRESS_1);
static TMC2209Stepper driver2(&MATCHING_SERIAL_PORT, R_SENSE, DRV_ADDRESS_2);

#define STEP_DELAY          200
#define STREAM_PLOT_DATA    1

#define STEPS_PER_RAD       63.66197f
#define MICROSTEPS_PER_STEP 32
#define MAX_ROT             3.14159f
#define GRAD_SCALE          0.025f
#define MAX_STEPSIZE        (PI / 36.0f)
#define MIN_STEPSIZE        (PI / 700.0f)

static const float kSampNum = 300.0f;

static float analogReadMilliVolts(int pin) {
  return (analogRead(pin) / 4095.0f) * 3300.0f;
}

static int round_up(float value) {
  int truncated = (int)value;
  return (value > truncated) ? truncated + 1 : truncated;
}

static float stepsToRad(int steps) {
  return steps / (STEPS_PER_RAD * MICROSTEPS_PER_STEP);
}

static int radToSteps(float rads) {
  return round_up(rads * (STEPS_PER_RAD * MICROSTEPS_PER_STEP));
}

static void takeStep(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY / 2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY / 2);
}

static float turnByRad(TMC2209Stepper &driver, int stepPin, int dirPin,
                       float rads, float &motor_pos, bool ignoreLimits = false) {
  (void)driver;
  if (rads == 0.0f) return 0.0f;

  bool  isNeg         = (rads < 0.0f);
  int   total_steps   = radToSteps(isNeg ? -rads : rads);
  float step_increment = stepsToRad(1);
  float pos_change    = isNeg ? -step_increment : step_increment;
  int   steps_taken   = 0;

  digitalWrite(dirPin, isNeg);

  for (int i = 0; i < total_steps; i++) {
    if (!ignoreLimits) {
      if ((motor_pos >= MAX_ROT && pos_change > 0.0f) ||
          (motor_pos <= 0.0f   && pos_change < 0.0f)) {
        Serial.println("MOTOR LIMIT REACHED");
        break;
      }
    }
    takeStep(stepPin);
    motor_pos += pos_change;
    steps_taken++;
  }
  return steps_taken * pos_change;
}

static float sampVSWR(int fwd, int rev) {
  float sum_fwd = 0.0f, sum_rev = 0.0f;
  for (int i = 0; i < (int)kSampNum; i++) {
    sum_fwd += analogReadMilliVolts(fwd);
    sum_rev += analogReadMilliVolts(rev);
    delayMicroseconds(15);
  }
  float avgFwd = sum_fwd / kSampNum;
  float avgRev = sum_rev / kSampNum;

  lastFwdV = avgFwd / 1000.0f;
  lastRevV = avgRev / 1000.0f;

  float denom = (avgFwd - avgRev);
  float vswr  = (denom != 0.0f) ? (avgFwd + avgRev) / denom : 99.0f;
  lastVSWR    = vswr;

  float loss = (vswr - 1.0f) * (vswr - 1.0f);

  if (vswr > 1.4f) atMatch = false;
  if (vswr < 1.2f) atMatch = true;

#if STREAM_PLOT_DATA
  Serial.print("VSWR_CSV,");
  Serial.print(millis());
  Serial.print(",");  Serial.print(vswr, 6);
  Serial.print(",");  Serial.print(lastFwdV, 6);
  Serial.print(",");  Serial.print(lastRevV, 6);
  Serial.print(",");  Serial.print(motor1_pos, 6);
  Serial.print(",");  Serial.print(motor2_pos, 6);
  Serial.print(",");  Serial.println(atMatch ? 1 : 0);
#endif

  return loss;
}

static float clampMagnitude(float value, float minMag, float maxMag) {
  float mag  = (value < 0.0f) ? -value : value;
  float sign = (value < 0.0f) ? -1.0f  : 1.0f;
  if (mag > maxMag) return maxMag * sign;
  if (mag < minMag) return minMag * sign;
  return value;
}

static void calcGradAndStep(TMC2209Stepper &driver, int stepPin, int dirPin,
                            float &gradient, float &motor_pos) {
  float initialCost   = sampVSWR(FWD_PIN, REV_PIN);
  float commandedStep = clampMagnitude(gradient * GRAD_SCALE, MIN_STEPSIZE, MAX_STEPSIZE);

  if (motor_pos >= MAX_ROT - MIN_STEPSIZE) commandedStep = -MIN_STEPSIZE;
  if (motor_pos <= MIN_STEPSIZE)           commandedStep =  MIN_STEPSIZE;

  float actualTravel = turnByRad(driver, stepPin, dirPin, commandedStep, motor_pos);
  delay(5);
  if (actualTravel != 0.0f)
    gradient = (initialCost - sampVSWR(FWD_PIN, REV_PIN)) / actualTravel;
}

void matching_init_motor_pins() {
  pinMode(STEP_PIN_1,   OUTPUT);
  pinMode(DIR_PIN_1,    OUTPUT);
  pinMode(STEP_PIN_2,   OUTPUT);
  pinMode(DIR_PIN_2,    OUTPUT);
  pinMode(TRANSMIT_PIN, OUTPUT);
  digitalWrite(TRANSMIT_PIN, HIGH);
}

void matching_init_uart() {
  MATCHING_SERIAL_PORT.begin(500000, SERIAL_8N1);
  delay(500);
}

void matching_homing() {
  Serial.println("Starting Homing Sequence...");
  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, -PI, motor1_pos, true);
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, -PI, motor2_pos, true);
  motor1_pos = 0.0f;
  motor2_pos = 0.0f;
  Serial.println("Finished Homing Sequence...");
  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, PI / 2.0f, motor1_pos, true);
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, PI / 2.0f, motor2_pos, true);

  motor1Pos = constrain((long)(motor1_pos * (180.0f / PI)), MOTOR_MIN_POS, MOTOR_MAX_POS);
  motor2Pos = constrain((long)(motor2_pos * (180.0f / PI)), MOTOR_MIN_POS, MOTOR_MAX_POS);
}

void matching_tick() {
  if (opMode != MODE_AUTO) return;
  if (!atMatch) {
    calcGradAndStep(driver1, STEP_PIN_1, DIR_PIN_1, dM1, motor1_pos);
    calcGradAndStep(driver2, STEP_PIN_2, DIR_PIN_2, dM2, motor2_pos);
  } else {
    sampVSWR(FWD_PIN, REV_PIN);
  }
  motor1Pos = constrain((long)(motor1_pos * (180.0f / PI)), MOTOR_MIN_POS, MOTOR_MAX_POS);
  motor2Pos = constrain((long)(motor2_pos * (180.0f / PI)), MOTOR_MIN_POS, MOTOR_MAX_POS);
}

float getVSWR()           { return lastVSWR; }
float getForwardVoltage() { return lastFwdV; }
float getReverseVoltage() { return lastRevV; }

void setMotor1Step(long posDeg) {
  float targetRad = posDeg * (PI / 180.0f);
  float delta     = targetRad - motor1_pos;
  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, delta, motor1_pos);
}

void setMotor2Step(long posDeg) {
  float targetRad = posDeg * (PI / 180.0f);
  float delta     = targetRad - motor2_pos;
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, delta, motor2_pos);
}

void setRadioTX(bool en) {
  digitalWrite(TRANSMIT_PIN, en ? LOW : HIGH);
}
