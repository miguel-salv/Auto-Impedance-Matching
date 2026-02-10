/*
 * SWR Meter Reader
 * * Hardware Setup:
 * - Sensors connected to voltage divider (2.2M / 1M)
 * - Divider otput buffered by op-amp
 * - Buffer Output -> 1k Resistor -> Teensy Pins A0/A1
 */

// Configuration Constants
const int PIN_FWD = A0;      // Pin 14
const int PIN_REV = A1;      // Pin 15
const int AVG_SAMPLES = 50;  // How many samples to average

// Calibration Constants
// Voltage Divider Ratio: (R1 + R2) / R2
// R1 = 2.2M, R2 = 1.0M -> (2.2 + 1.0) / 1.0 = 3.2
const float DIVIDER_RATIO = 3.2; 

// Teensy ADC Reference
const float V_REF = 3.3; // Teensy 4.1

// Op-Amp Offset
// Measure when SWR is 1.0 (input is 0V)
const float OPAMP_OFFSET = 0.03; 

void setup() {
  Serial.begin(115200);
  
  // Set ADC resolution to 12-bit
  analogReadResolution(12); // 4095 max value
  
  pinMode(PIN_FWD, INPUT);
  pinMode(PIN_REV, INPUT);
  
  delay(1000);
  Serial.println("SWR Meter Ready");
}

void loop() {
  // Read raw ADC Values
  float raw_fwd = readAverage(PIN_FWD);
  float raw_rev = readAverage(PIN_REV);

  // Convert to pin voltage (0 - 3.3V)
  float pin_volts_fwd = raw_fwd * (V_REF / 4095.0);
  float pin_volts_rev = raw_rev * (V_REF / 4095.0);

  // Convert to real sensor voltage (0 - 10V)
  float real_volts_fwd = (pin_volts_fwd * DIVIDER_RATIO) - OPAMP_OFFSET;
  float real_volts_rev = (pin_volts_rev * DIVIDER_RATIO) - OPAMP_OFFSET;

  // Clamp negative voltages to 0.0
  if (real_volts_fwd < 0) real_volts_fwd = 0.0;
  if (real_volts_rev < 0) real_volts_rev = 0.0;

  // Calculate SWR
  // SWR = (V_fwd + V_rev) / (V_fwd - V_rev)
  float swr = 0.0;
  
  if (real_volts_fwd > 0.5) { // Only calc if forward voltage > 0.5V
    if (real_volts_rev >= real_volts_fwd) {
       swr = 99.9; // Infinite SWR
    } else {
       // Reflection coefficient
       float gamma = real_volts_rev / real_volts_fwd;
       
       // SWR Calculation
       swr = (1.0 + gamma) / (1.0 - gamma);
    }
  } else {
    swr = 0.0; // Transmitter is off
  }

  Serial.print("V_Fwd:");
  Serial.print(real_volts_fwd, 2);
  Serial.print("  V_Rev:");
  Serial.print(real_volts_rev, 2);
  Serial.print("  SWR:");
  Serial.println(swr, 2);

  // Update rate (10Hz)
  delay(100); 
}

// Helper function to smooth out noise
float readAverage(int pin) {
  long sum = 0;
  for (int i = 0; i < AVG_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(10); // Pause between reads for ADC stability
  }
  return (float)sum / AVG_SAMPLES;
}