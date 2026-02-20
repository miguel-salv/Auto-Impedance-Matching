#include <Arduino.h>
#include <TMCStepper.h>

// --- PIN CONFIGURATION ---
const int STEP_PIN = D0;
const int DIR_PIN  = D1;
const int RX_PIN   = D7; // Xiao RX -> TMC TX
const int TX_PIN   = D6; // Xiao TX -> TMC RX

// --- TMC2209 UART CONFIGURATION ---
#define SERIAL_PORT Serial1       // Hardware Serial 1 on ESP32-S3
#define R_SENSE     0.11f         // Standard sense resistor on most TMC2209 modules
#define DRV_ADDRESS 0b00          // Default UART address for the driver

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRV_ADDRESS);

// --- MOTION & STALLGUARD SETTINGS ---
#define STALL_VALUE 100          // [0..255] Sensitivity. Lower = less sensitive. Needs tuning!
#define STEP_DELAY  200           // Microseconds between steps. Controls speed.

bool currentDirection = true;     // Tracks the current motor direction

unsigned long lastPollTime = 0;
const int POLL_INTERVAL = 10; // Read SG_RESULT every 50ms
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait up to 3 seconds for serial monitor
  
  Serial.println("Initializing TMC2209 Sensorless Bouncing...");

  // 1. Configure Hardware Pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, currentDirection);
  
  // 2. Initialize Hardware UART for the Xiao ESP32-S3
  SERIAL_PORT.begin(500000, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // 3. Configure TMC2209
  driver.begin();
  driver.toff(5);                 // Enable driver in software
  driver.rms_current(800);        // Set motor RMS current in mA
  driver.microsteps(16);          // Set microsteps to 1/16
  
  // 4. Configure StallGuard4
  driver.en_spreadCycle(false);   // MUST be false. StallGuard4 only works in StealthChop.
  driver.pwm_autoscale(true);     // Required for StealthChop to work properly.
  
  // TCOOLTHRS defines the lower velocity threshold for StallGuard. 
  // 0xFFFFF is a safe catch-all to enable it at almost all speeds for testing.
  driver.TCOOLTHRS(0xFFFFF);    
  
  // Set the stall sensitivity threshold
  driver.SGTHRS(STALL_VALUE);

  uint8_t conn_result = driver.test_connection();
  if (conn_result == 0) {
    Serial.println("UART connection successful!");
  } else {
    Serial.print("UART connection FAILED. Error code: ");
    Serial.println(conn_result);
    while(1); // Halt the program so you can fix the wiring
  }
  
  Serial.println("Setup Complete. Starting motor.");
}



void loop() {
  // 1. Uninterrupted Stepping
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY / 2);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY / 2);

  // 2. Timed UART Polling
  if (millis() - lastPollTime >= POLL_INTERVAL) {
    lastPollTime = millis();
    
    uint16_t sg_result = driver.SG_RESULT();

    // Only trigger if we actually hit 0 (a hard stall)
    if (sg_result < 250) {
      Serial.println("Stall detected! Bouncing back...");
      delay(50);
      
      currentDirection = !currentDirection;
      digitalWrite(DIR_PIN, currentDirection);
      
      // Force step away from the wall
      for(int i = 0; i < 50; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(STEP_DELAY / 2);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(STEP_DELAY / 2);
      }
      
      // Reset the timer so we don't immediately poll again
      lastPollTime = millis(); 
    }
  }
}