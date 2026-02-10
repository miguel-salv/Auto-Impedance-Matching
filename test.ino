/*
 * Verify both servos move
 */

#include <Servo.h>

const int tx_servoPin = 0;
const int ant_servoPin = 23;

Servo tx_servo;
Servo ant_servo;

void setup() {
  Serial.begin(9600);
  tx_servo.attach(tx_servoPin);
  ant_servo.attach(ant_servoPin);
  Serial.println("Test both servos");
}

void loop() {
  int angles[] = { 0, 90, 180, 90, 0 };
  for (int i = 0; i < 5; i++) {
    tx_servo.write(angles[i]);
    ant_servo.write(angles[i]);
    Serial.println(angles[i]);
    delay(1000);
  }
}
