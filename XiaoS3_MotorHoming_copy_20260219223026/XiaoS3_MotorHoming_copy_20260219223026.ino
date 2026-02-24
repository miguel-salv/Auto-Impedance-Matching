#include <Arduino.h>
#include <TMCStepper.h>

// --- PIN CONFIGURATION ---
const int STEP_PIN_1 = D2;
const int DIR_PIN_1 = D3;
const int STEP_PIN_2 = D8;
const int DIR_PIN_2 = D9;
const int RX_PIN = D7;
const int TX_PIN = D6;
const int FWD_PIN = D0;
const int REV_PIN = D1;

// --- TMC2209 UART CONFIGURATION ---
#define SERIAL_PORT Serial1
#define R_SENSE 0.11f
#define DRV_ADDRESS_1 0b00
#define DRV_ADDRESS_2 0b01

TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRV_ADDRESS_1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRV_ADDRESS_2);

// --- SETTINGS ---
#define STALL_VALUE 140
#define STEP_DELAY 200
#define POLL_INTERVAL 10


//--- MECH SPECS ---
#define STEPS_PER_RAD 63.66197f
#define MICROSTEPS_PER_STEP 16
#define MAX_ROT 3.14159f
#define GRAD_SCALE 0.005f

#define GRAD_DEADBAND .001f


#define PI 3.14159f

#define MAX_STEPSIZE PI/36
#define MIN_STEPSIZE PI/90
// --- POSITION TRACKING ---
float motor1_pos = 2 * PI;
float motor2_pos = 2 * PI;

bool dir1 = true;
bool dir2 = true;

float dM1 = 0.1;
float dM2 = 0.1;

float SWR;

const float samp_num = 1000.0;

float turnByRad(TMC2209Stepper &driver, int stepPin, int dirPin, float rads, float &motor_pos, bool ignoreLimits = false) {
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

void setup() {
  Serial.begin(500000);
  while (!Serial && millis() < 3000)
    ;

  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

  digitalWrite(DIR_PIN_1, dir1);
  digitalWrite(DIR_PIN_2, dir2);

  SERIAL_PORT.begin(500000, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(500);

  // Configure Driver 1
  driver1.begin();
  driver1.toff(5);
  driver1.rms_current(800);
  driver1.microsteps(16);
  driver1.en_spreadCycle(false);
  driver1.pwm_autoscale(true);
  driver1.TCOOLTHRS(0xFFFFF);
  driver1.SGTHRS(STALL_VALUE);

  // Configure Driver 2
  driver2.begin();
  driver2.toff(5);
  driver2.rms_current(800);
  driver2.microsteps(16);
  driver2.en_spreadCycle(false);
  driver2.pwm_autoscale(true);
  driver2.TCOOLTHRS(0xFFFFF);
  driver2.SGTHRS(STALL_VALUE);


  testUART(driver1, 1);
  testUART(driver2, 2);

  Serial.println("Starting Homing Sequence...");
  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, -1 * PI, motor1_pos, true);
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, -1 * PI, motor2_pos, true);

  motor1_pos = 0;
  motor2_pos = 0;
  Serial.println("Finished Homing Sequence...");
  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, PI/2, motor1_pos, true);
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, PI/2, motor2_pos, true);
}

void loop() {
  calcGradAndStep(driver1, STEP_PIN_1, DIR_PIN_1, dM1, motor1_pos);
  //Serial.print("motor1: ");
  //Serial.print(motor1_pos);
  calcGradAndStep(driver2, STEP_PIN_2, DIR_PIN_2, dM2, motor2_pos);
  //Serial.print(" motor2: ");
  //Serial.print(motor2_pos);

  //Serial.print(" Voltage Read D0:");
  //Serial.println(readPinVoltage(REV_PIN));
}


void calcGradAndStep(TMC2209Stepper &driver, int stepPin, int dirPin, float &gradient, float &motor_pos) {

  // if (gradient > -GRAD_DEADBAND && gradient < GRAD_DEADBAND) {
  //   return;
  // }

  float initialCost = sampVSWR(FWD_PIN, REV_PIN);
  Serial.println(gradient);
  
  float commandedStepSize = gradient * GRAD_SCALE;

  commandedStepSize = clampMagnitude(commandedStepSize, MIN_STEPSIZE, MAX_STEPSIZE);

  bool hittingUpperLimit = (motor_pos >= MAX_ROT-MIN_STEPSIZE) ;
  bool hittingLowerLimit = (motor_pos <= MIN_STEPSIZE);
  if(hittingUpperLimit){
    commandedStepSize = -MIN_STEPSIZE;
  }
  if(hittingLowerLimit){
    commandedStepSize = MIN_STEPSIZE;
  }


  float actualTravel = turnByRad(driver, stepPin, dirPin,commandedStepSize, motor_pos);

  delay(250);

  if (actualTravel != 0.0f) {
    gradient = (initialCost - sampVSWR(FWD_PIN, REV_PIN)) / actualTravel;
  }
}

float clampMagnitude(float value, float minMagnitude, float maxMagnitude) {
    float magnitude = (value < 0.0f) ? -value : value;
    float sign = (value < 0.0f) ? -1.0f : 1.0f;

    if (magnitude > maxMagnitude) {
        return maxMagnitude * sign;
    }

    if (magnitude < minMagnitude) {
        return minMagnitude * sign;
    }

    return value;
}

float calcSWR() {
  float err1 = motor1_pos - 2 * PI;
  float err2 = motor2_pos - (PI / 2.0f);
  return (err1 * err1) + (err2 * err2);
}
float stepsToRad(int steps) {
  return steps / (STEPS_PER_RAD * MICROSTEPS_PER_STEP);
}
int radToSteps(float rads) {
  return round_up(rads * (STEPS_PER_RAD * MICROSTEPS_PER_STEP));
}


void testUART(TMC2209Stepper &driver, int drivNum) {
  uint8_t conn_result = driver.test_connection();
  if (conn_result == 0) {
    Serial.print("UART connection successful: ");
    Serial.println(drivNum);
  } else {
    Serial.print("UART connection FAILED. Error code: ");
    Serial.println(conn_result);
    while (1)
      ;  // Halt the program so you can fix the wiring
  }
}

float sampVSWR(int fwd, int rev){
  float sum_fwd = 0.0;
  float sum_rev = 0.0;
  for (int i = 0; i < samp_num; i++) {
    // Read the factory-calibrated voltage in mV
    sum_fwd += analogReadMilliVolts(fwd);
    sum_rev += analogReadMilliVolts(rev);
    delayMicroseconds(30);
  }
  // Calculate Average Millivolts
  float averageMv_fwd = sum_fwd / samp_num;
  float averageMv_rev = sum_rev / samp_num;
  // Convert mV to Volts
  float result = (averageMv_fwd+averageMv_rev)/((averageMv_fwd-averageMv_rev));
  Serial.print(" VSWR:");
  Serial.println(result);
  return result;
}

void takeStep(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY / 2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY / 2);
}

float readPinVoltage(int sensorPin) {
  float sum = 0.0;
  for (int i = 0; i < samp_num; i++) {
    // Read the factory-calibrated voltage in mV
    sum += analogReadMilliVolts(sensorPin);
    delayMicroseconds(30);
  }
  // Calculate Average Millivolts
  float averageMv = sum / samp_num;
  // Convert mV to Volts
  float voltage = averageMv / 1000.0;
  return voltage;
}

int round_up(float value) {
  int truncated = static_cast<int>(value);

  // Casting to int truncates towards zero.
  // If the original float is strictly greater than the truncated int,
  // it had a positive fractional component that requires bumping up.
  if (value > truncated) {
    return truncated + 1;
  }

  return truncated;
}
