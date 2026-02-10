#include <Servo.h>

/**
 * This code has been developed as part of the Hackerfab Automatic Impedance
 * Matcher project, for Fall 2025. It controls two servos (tx_servo and
 * ant_servo) based on the position of two potentiometers in MANUAL mode, or
 * simulates automatic control based on a VSWR reading in AUTOMATED mode.
 *
 * @author Aiden Magee, Hackerfab
 */

// Define pins
const int tx_dialPin = 39;       // == 14. Potentiometer for tx_servo
const int ant_dialPin = 38;      // == 15. Potentiometer for ant_servo
const int switchPin = 32;        // Switch to toggle between states
const int tx_servoPin = 0;       // Control pin for tx_servo
const int ant_servoPin = 23;     // Control pin for ant_servo

// Create Servo objects
Servo tx_servo;
Servo ant_servo;

// Define states
enum State { AUTOMATED, MANUAL };
State state = MANUAL;  // Start in MANUAL mode

// Forward declarations
State parseSwitchState();
void controlServosAutomated();
void controlServosManual();
int get_vswr();

void setup() {
  tx_servo.attach(tx_servoPin);
  ant_servo.attach(ant_servoPin);

  Serial.begin(9600);

  pinMode(switchPin, INPUT_PULLUP);
}

void loop() {
  state = parseSwitchState();

  if (state == AUTOMATED) {
    controlServosAutomated();
  } else if (state == MANUAL) {
    controlServosManual();
  }
  delay(1000);
}

State parseSwitchState() {
  int switchState = digitalRead(switchPin);
  if (switchState == HIGH) {
    return MANUAL;
  } else {
    return AUTOMATED;
  }
}

void controlServosAutomated() {
  int vswr = get_vswr();
  int tx_angle = map(vswr, 0, 1023, 0, 180);

  tx_servo.write(tx_angle);
  ant_servo.write(0);

  Serial.print("AUTOMATED: vswr=");
  Serial.print(vswr);
  Serial.print(", tx_angle=");
  Serial.println(tx_angle);
}

void controlServosManual() {
  int tx_dial = analogRead(tx_dialPin);
  int ant_dial = analogRead(ant_dialPin);

  int tx_angle = map(tx_dial, 0, 1023, 0, 180);
  int ant_angle = map(ant_dial, 0, 1023, 0, 180);

  tx_servo.write(tx_angle);
  ant_servo.write(ant_angle);

  Serial.print("MANUAL: tx_dial=");
  Serial.print(tx_dial);
  Serial.print(", tx_angle=");
  Serial.print(tx_angle);
  Serial.print(", ant_dial=");
  Serial.print(ant_dial);
  Serial.print(", ant_angle=");
  Serial.println(ant_angle);
}

int prev_vswr = 0;
int get_vswr() {
  prev_vswr = (prev_vswr + 3) % 100;
  return prev_vswr;
}
