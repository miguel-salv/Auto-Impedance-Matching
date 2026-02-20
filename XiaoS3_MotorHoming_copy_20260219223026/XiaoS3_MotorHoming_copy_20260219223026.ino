#include <Arduino.h>
#include <TMCStepper.h>

// --- PIN CONFIGURATION ---
const int STEP_PIN_1 = D0;
const int DIR_PIN_1  = D1;
const int STEP_PIN_2 = D8;
const int DIR_PIN_2  = D9;
const int RX_PIN     = D7; 
const int TX_PIN     = D6; 

// --- TMC2209 UART CONFIGURATION ---
#define SERIAL_PORT Serial1       
#define R_SENSE     0.11f         
#define DRV_ADDRESS_1 0b00 
#define DRV_ADDRESS_2 0b01 

TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, DRV_ADDRESS_1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRV_ADDRESS_2);

// --- SETTINGS ---
#define STALL_VALUE 100           
#define STEP_DELAY  200           
#define POLL_INTERVAL 50          

bool dir1 = true;
bool dir2 = true;
bool sequenceComplete = false;

void setup() {
  Serial.begin(500000);
  while (!Serial && millis() < 3000); 
  
  pinMode(STEP_PIN_1, OUTPUT); pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT); pinMode(DIR_PIN_2, OUTPUT);
  
  digitalWrite(DIR_PIN_1, dir1); 
  digitalWrite(DIR_PIN_2, dir2);

  // Initialize UART at 500k as per your successful test
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

  Serial.println("Setup Complete. Starting Sequence...");
}

void loop() {

  Serial.println("Homing Motor 1...");
  runUntilStall(driver1, STEP_PIN_1, DIR_PIN_1, dir1,270);
  
  delay(1000); // Small gap between motors for clarity

  Serial.println("Homing Motor 2...");
  runUntilStall(driver2, STEP_PIN_2, DIR_PIN_2, dir2,250);

  Serial.println("Sequence Complete.");

}

// Dedicated function to run a motor until stall, then bounce
void runUntilStall(TMC2209Stepper &driver, int stepPin, int dirPin, bool &currentDir, int threshold) {
  unsigned long lastPoll = 0;
  bool stalled = false;
  //get the motor started
  for(int i = 0; i < 50; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY / 2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY / 2);
  }
  while (!stalled) {
    
    // 1. Physical Step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY / 2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY / 2);
    //Serial.println(driver.SG_RESULT() );
    // 2. Check StallGuard every 10ms
    if (millis() - lastPoll >= POLL_INTERVAL) {
      int pollres = driver.SG_RESULT();
      Serial.println(pollres);
      lastPoll = millis();
      if (pollres < threshold) { 
        
        stalled = true;
      }
    }
  }

  // 3. Bounce Logic (Inherited from your original script)
  Serial.println("Stall detected! Bouncing...");
  delay(50); // Resonance settle [cite: 62]
  
  currentDir = !currentDir;
  digitalWrite(dirPin, currentDir);
  
  // Force 50 steps away from wall [cite: 68]
  for(int i = 0; i < 50; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY / 2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY / 2);
  }
}