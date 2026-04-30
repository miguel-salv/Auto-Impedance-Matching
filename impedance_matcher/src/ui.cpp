#include "../include/ui.h"
#include "../include/pins_config.h"
#include "../include/app_state.h"
#include "../include/matching.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cstring>

static Adafruit_SSD1306 g_oled(SCREEN_W, SCREEN_H, &Wire, -1);

static uint32_t lastDisplayMs = 0;

static int  menuSel          = 0;
static int  homeSel          = 0;
static bool menuEditingMotor = false;
static int  editingMotorId   = -1;

#define MOTOR_STEP_SIZE 10

static int buildMenu(int* out) {
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

static const char* menuLabel(int id) {
  switch (id) {
    case M_MODE:        return "Mode";
    case M_MOTOR1:      return "Motor 1";
    case M_MOTOR2:      return "Motor 2";
    case M_ADV_METRICS: return "Advanced";
    case M_BACK:        return "< Back";
    default:            return "?";
  }
}

static const char* menuValue(int id) {
  switch (id) {
    case M_MODE: return (opMode == MODE_AUTO) ? "AUTO" : "MANUAL";
    default:     return nullptr;
  }
}

static void drawHome() {
  g_oled.clearDisplay();
  g_oled.setTextColor(SSD1306_WHITE);

  g_oled.setTextSize(1);
  g_oled.setCursor(0, 0);
  g_oled.print("Impedance Matcher");
  g_oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  g_oled.setTextSize(1);
  g_oled.setCursor(0, 13);
  g_oled.print("VSWR");
  g_oled.setTextSize(2);
  g_oled.setCursor(0, 23);
  g_oled.print(getVSWR(), 3);

  g_oled.setTextSize(1);
  const char* modeStr = (opMode == MODE_AUTO) ? "AUTO" : "MANUAL";
  int modeLabelX = SCREEN_W - ((int)strlen("Mode:") * 6);
  int modeValueX = SCREEN_W - ((int)strlen(modeStr) * 6);
  g_oled.setCursor(modeLabelX, 13);
  g_oled.print("Mode:");
  g_oled.setCursor(modeValueX, 22);
  g_oled.print(modeStr);

  const int btnY = 49, btnW = 60, btnH = 14, gap = 8;
  for (int i = 0; i < 2; i++) {
    int  x   = i * (btnW + gap);
    bool sel = (homeSel == i);
    if (sel) {
      g_oled.fillRect(x, btnY, btnW, btnH, SSD1306_WHITE);
      g_oled.setTextColor(SSD1306_BLACK);
    } else {
      g_oled.drawRect(x, btnY, btnW, btnH, SSD1306_WHITE);
      g_oled.setTextColor(SSD1306_WHITE);
    }
    g_oled.setCursor(x + 4, btnY + 4);
    g_oled.print(i == 0 ? (radioTX ? "TX:ON" : "TX:OFF") : "Settings");
  }

  g_oled.display();
}

static void drawMenu() {
  int visible[M_COUNT];
  int count = buildMenu(visible);
  menuSel = constrain(menuSel, 0, count - 1);

  g_oled.clearDisplay();
  g_oled.setTextColor(SSD1306_WHITE);
  g_oled.setTextSize(1);
  g_oled.setCursor(0, 0);
  g_oled.print("Settings");
  g_oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  const int ROWS  = 4;
  int       start = constrain(menuSel - ROWS + 1, 0, max(0, count - ROWS));

  for (int i = 0; i < ROWS && (start + i) < count; i++) {
    int  idx = start + i;
    int  id  = visible[idx];
    int  y   = 12 + i * 13;
    bool sel = (idx == menuSel);

    if (sel) {
      g_oled.fillRect(0, y - 1, 127, 12, SSD1306_WHITE);
      g_oled.setTextColor(SSD1306_BLACK);
    } else {
      g_oled.setTextColor(SSD1306_WHITE);
    }

    g_oled.setCursor(3, y);
    g_oled.print(menuLabel(id));

    g_oled.setCursor(80, y);
    if (id == M_MOTOR1) {
      g_oled.print(motor1Pos);
      if (menuEditingMotor && editingMotorId == M_MOTOR1) g_oled.print("*");
    } else if (id == M_MOTOR2) {
      g_oled.print(motor2Pos);
      if (menuEditingMotor && editingMotorId == M_MOTOR2) g_oled.print("*");
    } else {
      const char* val = menuValue(id);
      if (val) g_oled.print(val);
    }
  }

  g_oled.setTextColor(SSD1306_WHITE);
  g_oled.display();
}

static void drawMotorAdjust(int motorNum, long pos) {
  g_oled.clearDisplay();
  g_oled.setTextColor(SSD1306_WHITE);
  g_oled.setTextSize(1);
  g_oled.setCursor(0, 0);
  g_oled.print("Motor ");
  g_oled.print(motorNum);
  g_oled.print(" (deg)");
  g_oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  g_oled.setTextSize(2);
  g_oled.setCursor(4, 20);
  g_oled.print("< ");
  g_oled.print(pos);
  g_oled.print(" >");

  g_oled.setTextSize(1);
  g_oled.setCursor(0, 50);
  g_oled.print("Turn:adj  Press:back");
  g_oled.display();
}

static void drawAdvancedMetrics() {
  bool m1Max = (motor1Pos <= MOTOR_MIN_POS) || (motor1Pos >= MOTOR_MAX_POS);
  bool m2Max = (motor2Pos <= MOTOR_MIN_POS) || (motor2Pos >= MOTOR_MAX_POS);

  g_oled.clearDisplay();
  g_oled.setTextColor(SSD1306_WHITE);
  g_oled.setTextSize(1);
  g_oled.setCursor(0, 0);
  g_oled.print("Advanced Metrics");
  g_oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  g_oled.setCursor(0, 12);
  g_oled.print("VSWR:");
  g_oled.print(getVSWR(), 3);

  g_oled.setCursor(0, 22);
  g_oled.print("FWD:");
  g_oled.print(getForwardVoltage(), 3);
  g_oled.print("V REV:");
  g_oled.print(getReverseVoltage(), 3);
  g_oled.print("V");

  g_oled.setCursor(0, 32);
  g_oled.print("M1:");
  g_oled.print(motor1_pos * (180.0f / PI), 1);
  g_oled.print("d ");
  g_oled.print(m1Max ? "MAX" : "OK");

  g_oled.setCursor(0, 42);
  g_oled.print("M2:");
  g_oled.print(motor2_pos * (180.0f / PI), 1);
  g_oled.print("d ");
  g_oled.print(m2Max ? "MAX" : "OK");

  g_oled.setCursor(0, 54);
  g_oled.print("Press: back");
  g_oled.display();
}

static void handleHome(int delta, bool pressed) {
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

static void handleMenu(int delta, bool pressed) {
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
        if (opMode == MODE_AUTO) { menuEditingMotor = false; editingMotorId = -1; }
        { int v2[M_COUNT]; menuSel = constrain(menuSel, 0, buildMenu(v2) - 1); }
        break;
      case M_MOTOR1:      menuEditingMotor = true; editingMotorId = M_MOTOR1; break;
      case M_MOTOR2:      menuEditingMotor = true; editingMotorId = M_MOTOR2; break;
      case M_ADV_METRICS: state = S_METRICS; break;
      case M_BACK:
        menuEditingMotor = false;
        editingMotorId   = -1;
        state = S_HOME;
        break;
    }
  }
}

static void handleMotorAdjust(int motorNum, long& pos, int delta, bool pressed) {
  if (delta != 0) {
    pos += (long)delta * MOTOR_STEP_SIZE;
    pos = constrain(pos, MOTOR_MIN_POS, MOTOR_MAX_POS);
    if (motorNum == 1) setMotor1Step(pos);
    else               setMotor2Step(pos);
  }
  if (pressed) state = S_MENU;
}

static void handleMetrics(int delta, bool pressed) {
  (void)delta;
  if (pressed) state = S_MENU;
}

bool ui_init_display() {
  Wire.begin();
  if (!g_oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) return false;
  g_oled.clearDisplay();
  g_oled.display();
  return true;
}

bool ui_should_redraw(bool userInput, bool inAutoHome) {
  const bool slowRefreshDue =
      (lastDisplayMs == 0) ||
      ((millis() - lastDisplayMs) >= DISPLAY_THROTTLE_AUTO_HOME_MS);
  return userInput || !inAutoHome || slowRefreshDue;
}

void ui_tick(int delta, bool pressed, bool doDraw) {
  switch (state) {
    case S_HOME:
      handleHome(delta, pressed);
      if (doDraw) {
        drawHome();
        lastDisplayMs = millis();
      }
      break;
    case S_MENU:
      handleMenu(delta, pressed);
      if (doDraw) {
        drawMenu();
        lastDisplayMs = millis();
      }
      break;
    case S_MOTOR1:
      handleMotorAdjust(1, motor1Pos, delta, pressed);
      if (doDraw) {
        drawMotorAdjust(1, motor1Pos);
        lastDisplayMs = millis();
      }
      break;
    case S_MOTOR2:
      handleMotorAdjust(2, motor2Pos, delta, pressed);
      if (doDraw) {
        drawMotorAdjust(2, motor2Pos);
        lastDisplayMs = millis();
      }
      break;
    case S_METRICS:
      handleMetrics(delta, pressed);
      if (doDraw) {
        drawAdvancedMetrics();
        lastDisplayMs = millis();
      }
      break;
  }
}
