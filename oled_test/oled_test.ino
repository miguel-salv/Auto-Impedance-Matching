#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TMCStepper.h>

#define ENC_A    38
#define ENC_B    39
#define ENC_BTN  40

#define SCROLL_DIR -1  // +1 normal, -1 reversed
#define MOTOR_MIN_POS 0
#define MOTOR_MAX_POS 180
#define ENC_COUNTS_PER_CLICK 2 // lower for more sensitivity

#define TRANSMIT_PIN 31
#define STEP_PIN_1 2
#define DIR_PIN_1 3
#define STEP_PIN_2 9
#define DIR_PIN_2 10
#define FWD_PIN 25
#define REV_PIN 24

#define SERIAL_PORT Serial1
#define R_SENSE 0.11f
#define DRV_ADDRESS_1 0b00
#define DRV_ADDRESS_2 0b01

#define STALL_VALUE 140
#define STEP_DELAY 200

#define STEPS_PER_RAD 63.66197f
#define MICROSTEPS_PER_STEP 16
#define MAX_ROT 3.14159f
#define GRAD_SCALE 0.005f
#define MAX_STEPSIZE PI / 36
#define MIN_STEPSIZE PI / 180

const float samp_num = 1000.0f;

#define SCREEN_W 128
#define SCREEN_H  64
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);

enum AppState { S_HOME, S_MENU, S_MOTOR1, S_MOTOR2, S_METRICS };
enum OpMode   { MODE_AUTO, MODE_MANUAL };

enum MenuID {
  M_MODE   = 0,
  M_MOTOR1 = 1,
  M_MOTOR2 = 2,
  M_ADV_METRICS = 3,
  M_BACK   = 4,
  M_COUNT  = 5  
};

AppState state     = S_HOME;
OpMode   opMode    = MODE_AUTO;
bool     radioTX   = false;
long     motor1Pos = 0;
long     motor2Pos = 0;
float    motor1PosRad = PI / 2.0f;
float    motor2PosRad = PI / 2.0f;
float    dM1 = 0.1f;
float    dM2 = 0.1f;

TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRV_ADDRESS_1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRV_ADDRESS_2);

volatile int encRaw = 0;
int encAccum = 0;
int lastEncA = HIGH;

void readEncoder() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  if (a != lastEncA) {
    encRaw += (a == b) ? -1 : 1;
    lastEncA = a;
  }
}



int consumeDelta() {
  noInterrupts();
  int d = encRaw;
  encRaw = 0;
  interrupts();

  encAccum += d;
  int clicks = encAccum / ENC_COUNTS_PER_CLICK;
  encAccum -= clicks * ENC_COUNTS_PER_CLICK;
  return clicks * SCROLL_DIR;
}

#define DEBOUNCE_MS 40
bool     lastBtnRaw = HIGH;
bool     btnPending = false;
uint32_t lastBounce = 0;

void pollButton() {
  bool raw = digitalRead(ENC_BTN);
  if (raw == LOW && lastBtnRaw == HIGH && (millis() - lastBounce) > DEBOUNCE_MS) {
    btnPending = true;
    lastBounce = millis();
  }
  lastBtnRaw = raw;
}

bool consumeButton() {
  if (btnPending) { btnPending = false; return true; }
  return false;
}

int round_up(float value) {
  int truncated = static_cast<int>(value);
  if (value > truncated) return truncated + 1;
  return truncated;
}

float stepsToRad(int steps) {
  return steps / (STEPS_PER_RAD * MICROSTEPS_PER_STEP);
}

int radToSteps(float rads) {
  return round_up(rads * (STEPS_PER_RAD * MICROSTEPS_PER_STEP));
}

void takeStep(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY / 2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY / 2);
}

float analogReadMilliVolts(int pin) {
  int rawValue = analogRead(pin);
  return (rawValue / 4095.0f) * 3300.0f;
}

float readPinVoltage(int sensorPin) {
  float sum = 0.0f;
  for (int i = 0; i < samp_num; i++) {
    sum += analogReadMilliVolts(sensorPin);
    delayMicroseconds(30);
  }
  return (sum / samp_num) / 1000.0f;
}

float sampVSWR(int fwd, int rev) {
  float sum_fwd = 0.0f;
  float sum_rev = 0.0f;
  for (int i = 0; i < samp_num; i++) {
    sum_fwd += analogReadMilliVolts(fwd);
    sum_rev += analogReadMilliVolts(rev);
    delayMicroseconds(15);
  }
  float averageMv_fwd = sum_fwd / samp_num;
  float averageMv_rev = sum_rev / samp_num;
  return (averageMv_fwd + averageMv_rev) / (averageMv_fwd - averageMv_rev);
}

float clampMagnitude(float value, float minMagnitude, float maxMagnitude) {
  float magnitude = (value < 0.0f) ? -value : value;
  float sign = (value < 0.0f) ? -1.0f : 1.0f;
  if (magnitude > maxMagnitude) return maxMagnitude * sign;
  if (magnitude < minMagnitude) return minMagnitude * sign;
  return value;
}

float turnByRad(TMC2209Stepper &driver, int stepPin, int dirPin, float rads, float &motor_pos, bool ignoreLimits = false) {
  (void)driver;
  if (rads == 0.0f) return 0.0f;

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
      if (hittingUpperLimit || hittingLowerLimit) break;
    }
    takeStep(stepPin);
    motor_pos += position_change;
    steps_taken++;
  }
  return steps_taken * position_change;
}

void calcGradAndStep(TMC2209Stepper &driver, int stepPin, int dirPin, float &gradient, float &motor_pos) {
  float initialCost = sampVSWR(FWD_PIN, REV_PIN);
  float commandedStepSize = clampMagnitude(gradient * GRAD_SCALE, MIN_STEPSIZE, MAX_STEPSIZE);
  bool hittingUpperLimit = (motor_pos >= MAX_ROT - MIN_STEPSIZE);
  bool hittingLowerLimit = (motor_pos <= MIN_STEPSIZE);
  if (hittingUpperLimit) commandedStepSize = -MIN_STEPSIZE;
  if (hittingLowerLimit) commandedStepSize = MIN_STEPSIZE;

  float actualTravel = turnByRad(driver, stepPin, dirPin, commandedStepSize, motor_pos);
  delay(25);
  if (actualTravel != 0.0f) {
    gradient = (initialCost - sampVSWR(FWD_PIN, REV_PIN)) / actualTravel;
  }
}

void testUART(TMC2209Stepper &driver, int drivNum) {
  uint8_t conn_result = driver.test_connection();
  if (conn_result == 0) {
    Serial.print("UART connection successful: ");
    Serial.println(drivNum);
  } else {
    Serial.print("UART connection FAILED. Error code: ");
    Serial.println(conn_result);
    while (1) {}
  }
}

void moveMotorToDeg(TMC2209Stepper &driver, int stepPin, int dirPin, float &motorPosRadRef, long targetDeg) {
  float targetRad = constrain((float)targetDeg, (float)MOTOR_MIN_POS, (float)MOTOR_MAX_POS) * (PI / 180.0f);
  float delta = targetRad - motorPosRadRef;
  turnByRad(driver, stepPin, dirPin, delta, motorPosRadRef);
}

float getVSWR()           { return sampVSWR(FWD_PIN, REV_PIN); }
float getForwardVoltage() { return readPinVoltage(FWD_PIN); }
float getReverseVoltage() { return readPinVoltage(REV_PIN); }
void  setMotor1Step(long pos) { moveMotorToDeg(driver1, STEP_PIN_1, DIR_PIN_1, motor1PosRad, pos); }
void  setMotor2Step(long pos) { moveMotorToDeg(driver2, STEP_PIN_2, DIR_PIN_2, motor2PosRad, pos); }

void  setRadioTX(bool en) {
  if (en) digitalWrite(TRANSMIT_PIN, HIGH);
  else digitalWrite(TRANSMIT_PIN, LOW);
}

int menuSel = 0;
int homeSel = 0;  // 0 = TX toggle, 1 = Settings
bool menuEditingMotor = false;
int  editingMotorId   = -1;
#define MOTOR_STEP_SIZE 10  // steps per encoder detent

int buildMenu(int* out) {
  int n = 0;
  out[n++] = M_MODE;
  if (opMode == MODE_MANUAL) {
    out[n++] = M_MOTOR1;
    out[n++] = M_MOTOR2;
  }
  out[n++] = M_ADV_METRICS;
  out[n++] = M_BACK;
  return n;
}

const char* menuLabel(int id) {
  switch (id) {
    case M_MODE:   return "Mode";
    case M_MOTOR1: return "Motor 1";
    case M_MOTOR2: return "Motor 2";
    case M_ADV_METRICS: return "Advanced";
    case M_BACK:   return "< Back";
    default:       return "?";
  }
}

const char* menuValue(int id) {
  switch (id) {
    case M_MODE:  return (opMode == MODE_AUTO) ? "AUTO" : "MANUAL";
    default:      return nullptr;
  }
}

void drawHome() {
  float vswr = getVSWR();
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("Impedance Matcher");
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 13);
  oled.print("VSWR");
  oled.setTextSize(2);
  oled.setCursor(0, 23);
  oled.print(vswr, 3);

  oled.setTextSize(1);
  const char* modeStr = (opMode == MODE_AUTO) ? "AUTO" : "MANUAL";
  int modeLabelX = SCREEN_W - ((int)strlen("Mode") * 6);
  int modeValueX = SCREEN_W - ((int)strlen(modeStr) * 6);
  oled.setCursor(modeLabelX, 13);
  oled.print("Mode");
  oled.setCursor(modeValueX, 22);
  oled.print(modeStr);

  const int btnY = 49;
  const int btnW = 60;
  const int btnH = 14;
  const int gap  = 8;
  for (int i = 0; i < 2; i++) {
    int x = i * (btnW + gap);
    bool selected = (homeSel == i);
    if (selected) {
      oled.fillRect(x, btnY, btnW, btnH, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
    } else {
      oled.drawRect(x, btnY, btnW, btnH, SSD1306_WHITE);
      oled.setTextColor(SSD1306_WHITE);
    }
    oled.setCursor(x + 4, btnY + 4);
    if (i == 0) {
      oled.print(radioTX ? "TX:ON" : "TX:OFF");
    } else {
      oled.print("Settings");
    }
  }

  oled.display();
}

void drawMenu() {
  int visible[M_COUNT];
  int count = buildMenu(visible);
  menuSel = constrain(menuSel, 0, count - 1);

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("Settings");
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  const int ROWS  = 4;
  int       start = constrain(menuSel - ROWS + 1, 0, max(0, count - ROWS));

  for (int i = 0; i < ROWS && (start + i) < count; i++) {
    int  idx = start + i;
    int  id  = visible[idx];
    int  y   = 12 + i * 13;
    bool sel = (idx == menuSel);

    if (sel) {
      oled.fillRect(0, y - 1, 127, 12, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK);
    } else {
      oled.setTextColor(SSD1306_WHITE);
    }

    oled.setCursor(3, y);
    oled.print(menuLabel(id));

    oled.setCursor(80, y);
    if (id == M_MOTOR1) {
      oled.print(motor1Pos);
      if (menuEditingMotor && editingMotorId == M_MOTOR1) oled.print("*");
    } else if (id == M_MOTOR2) {
      oled.print(motor2Pos);
      if (menuEditingMotor && editingMotorId == M_MOTOR2) oled.print("*");
    } else {
      const char* val = menuValue(id);
      if (val) oled.print(val);
    }
  }

  oled.setTextColor(SSD1306_WHITE);
  oled.display();
}

void drawMotorAdjust(int motorNum, long pos) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("Motor ");
  oled.print(motorNum);
  oled.print(" (deg)");
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  oled.setTextSize(2);
  oled.setCursor(4, 20);
  oled.print("< ");
  oled.print(pos);
  oled.print(" >");

  oled.setTextSize(1);
  oled.setCursor(0, 50);
  oled.print("Turn:adj  Press:back");

  oled.display();
}

void drawAdvancedMetrics() {
  float vswr = getVSWR();
  float vFwd = getForwardVoltage();
  float vRev = getReverseVoltage();
  float m1Rot = motor1Pos / 360.0f;
  float m2Rot = motor2Pos / 360.0f;
  bool  m1Max = (motor1Pos <= MOTOR_MIN_POS) || (motor1Pos >= MOTOR_MAX_POS);
  bool  m2Max = (motor2Pos <= MOTOR_MIN_POS) || (motor2Pos >= MOTOR_MAX_POS);

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("Advanced Metrics");
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  oled.setCursor(0, 12);
  oled.print("VSWR:");
  oled.print(vswr, 3);

  oled.setCursor(0, 22);
  oled.print("FWD:");
  oled.print(vFwd, 3);
  oled.print("V REV:");
  oled.print(vRev, 3);
  oled.print("V");

  oled.setCursor(0, 32);
  oled.print("M1:");
  oled.print(m1Rot, 2);
  oled.print("r ");
  oled.print(m1Max ? "MAX" : "OK");

  oled.setCursor(0, 42);
  oled.print("M2:");
  oled.print(m2Rot, 2);
  oled.print("r ");
  oled.print(m2Max ? "MAX" : "OK");

  oled.setCursor(0, 54);
  oled.print("Press: back");
  oled.display();
}

void handleHome(int delta, bool pressed) {
  if (delta != 0) {
    homeSel = (homeSel + delta) % 2;
    if (homeSel < 0) homeSel += 2;
  }
  if (pressed) {
    if (homeSel == 0) {
      radioTX = !radioTX;
      setRadioTX(radioTX);
    } else {
      state   = S_MENU;
      menuSel = 0;
    }
  }
}

void handleMenu(int delta, bool pressed) {
  int visible[M_COUNT];
  int count = buildMenu(visible);

  if (delta != 0) {
    if (menuEditingMotor) {
      if (editingMotorId == M_MOTOR1) {
        motor1Pos += (long)delta * MOTOR_STEP_SIZE;
        motor1Pos = constrain(motor1Pos, MOTOR_MIN_POS, MOTOR_MAX_POS);
        setMotor1Step(motor1Pos);
      } else if (editingMotorId == M_MOTOR2) {
        motor2Pos += (long)delta * MOTOR_STEP_SIZE;
        motor2Pos = constrain(motor2Pos, MOTOR_MIN_POS, MOTOR_MAX_POS);
        setMotor2Step(motor2Pos);
      }
    } else if (count > 0) {
      menuSel = (menuSel + delta) % count;
      if (menuSel < 0) menuSel += count;
    }
  }

  if (pressed) {
    if (menuEditingMotor) {
      menuEditingMotor = false;
      editingMotorId   = -1;
      return;
    }

    int sel = visible[menuSel];
    switch (sel) {
      case M_MODE:
        opMode = (opMode == MODE_AUTO) ? MODE_MANUAL : MODE_AUTO;
        if (opMode == MODE_AUTO) {
          menuEditingMotor = false;
          editingMotorId   = -1;
        }
        { int v2[M_COUNT]; menuSel = constrain(menuSel, 0, buildMenu(v2) - 1); }
        break;
      case M_MOTOR1:
        menuEditingMotor = true;
        editingMotorId   = M_MOTOR1;
        break;
      case M_MOTOR2:
        menuEditingMotor = true;
        editingMotorId   = M_MOTOR2;
        break;
      case M_ADV_METRICS:
        state = S_METRICS;
        break;
      case M_BACK:
        menuEditingMotor = false;
        editingMotorId   = -1;
        state = S_HOME;
        break;
    }
  }
}

void handleMotorAdjust(int motorNum, long& pos, int delta, bool pressed) {
  if (delta != 0) {
    pos += (long)delta * MOTOR_STEP_SIZE;
    pos = constrain(pos, MOTOR_MIN_POS, MOTOR_MAX_POS);
    if (motorNum == 1) setMotor1Step(pos);
    else               setMotor2Step(pos);
  }
  if (pressed) {
    state = S_MENU;
  }
}

void handleMetrics(int delta, bool pressed) {
  (void)delta;
  if (pressed) state = S_MENU;
}

void setup() {
  analogReadResolution(12);
  Serial.begin(500000);
  while (!Serial && millis() < 3000) {}

  pinMode(ENC_A,   INPUT_PULLUP);
  pinMode(ENC_B,   INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(TRANSMIT_PIN, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

  SERIAL_PORT.begin(500000, SERIAL_8N1);
  delay(500);

  driver1.begin();
  driver1.toff(5);
  driver1.rms_current(800);
  driver1.microsteps(16);
  driver1.en_spreadCycle(false);
  driver1.pwm_autoscale(true);
  driver1.TCOOLTHRS(0xFFFFF);
  driver1.SGTHRS(STALL_VALUE);

  driver2.begin();
  driver2.toff(5);
  driver2.rms_current(800);
  driver2.microsteps(16);
  driver2.en_spreadCycle(false);
  driver2.pwm_autoscale(true);
  driver2.TCOOLTHRS(0xFFFFF);
  driver2.SGTHRS(STALL_VALUE);

  // testUART(driver1, 1);
  // testUART(driver2, 2);

  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, -PI, motor1PosRad, true);
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, -PI, motor2PosRad, true);
  motor1PosRad = 0.0f;
  motor2PosRad = 0.0f;
  turnByRad(driver1, STEP_PIN_1, DIR_PIN_1, PI / 2, motor1PosRad, true);
  turnByRad(driver2, STEP_PIN_2, DIR_PIN_2, PI / 2, motor2PosRad, true);
  motor1Pos = 90;
  motor2Pos = 90;

  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);

  Wire.begin();
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  oled.clearDisplay();
  oled.display();
}

void loop() {
  if (opMode == MODE_AUTO) {
    calcGradAndStep(driver1, STEP_PIN_1, DIR_PIN_1, dM1, motor1PosRad);
    calcGradAndStep(driver2, STEP_PIN_2, DIR_PIN_2, dM2, motor2PosRad);
    motor1Pos = (long)(motor1PosRad * 180.0f / PI);
    motor2Pos = (long)(motor2PosRad * 180.0f / PI);
    motor1Pos = constrain(motor1Pos, MOTOR_MIN_POS, MOTOR_MAX_POS);
    motor2Pos = constrain(motor2Pos, MOTOR_MIN_POS, MOTOR_MAX_POS);
  }

  pollButton();
  int  delta   = consumeDelta();
  bool pressed = consumeButton();

  switch (state) {
    case S_HOME:
      handleHome(delta, pressed);
      drawHome();
      break;
    case S_MENU:
      handleMenu(delta, pressed);
      drawMenu();
      break;
    case S_MOTOR1:
      handleMotorAdjust(1, motor1Pos, delta, pressed);
      drawMotorAdjust(1, motor1Pos);
      break;
    case S_MOTOR2:
      handleMotorAdjust(2, motor2Pos, delta, pressed);
      drawMotorAdjust(2, motor2Pos);
      break;
    case S_METRICS:
      handleMetrics(delta, pressed);
      drawAdvancedMetrics();
      break;
  }

  delay(16);
}
