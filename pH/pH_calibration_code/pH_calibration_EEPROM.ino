#include <Wire.h>
#include <EEPROM.h>  // Include EEPROM library for storing calibration data

const int pHSensorPin = A4; // Pin connected to the pH sensor's analog output

// Calibration data
float calibration_pH[2] = {4.0, 7.0};     // Known pH values for calibration
float calibration_raw[2] = {0.0, 0.0};    // Raw sensor values for calibration
bool isCalibrated = false;

float slope = 0.0;
float intercept = 0.0;

// EEPROM addresses
#define EEPROM_ADDR_SLOPE 0
#define EEPROM_ADDR_INTERCEPT 4

void setup() {
  // Start serial communication
  Serial.begin(9600);
  while (!Serial) { ; } // Wait for serial port to connect (for native USB)

  Serial.println("DFRobot Gravity pH Meter Pro V2");
  Serial.println("Connecting to pH sensor...");
  
  delay(2000); // Wait for sensor to stabilize
  
  // Load calibration data from EEPROM
  EEPROM.get(EEPROM_ADDR_SLOPE, slope);
  EEPROM.get(EEPROM_ADDR_INTERCEPT, intercept);
  
  if (slope != 0.0 && intercept != 0.0) {
    isCalibrated = true;
    Serial.println("Calibration data loaded from EEPROM.");
  } else {
    Serial.println("No calibration data found. Please calibrate by entering 'cal'.");
  }
  
  Serial.println("Enter 'cal' to start calibration.");
}

void loop() {
  handleSerialInput(); // Check for calibration command

  if (isCalibrated) {
    float pH = readPHValue();
    Serial.print("Current pH: ");
    Serial.println(pH, 2);
  } else {
    Serial.println("Sensor not calibrated. Please calibrate first by entering 'cal'.");
  }
  
  delay(1000); // Wait for 1 second before next reading
}

void startCalibration() {
  Serial.println("Starting calibration process...");
  
  // Calibration Point 1: pH 4.0
  Serial.println("1. Immerse the pH sensor in pH 4.00 solution and press any key.");
  waitForUser();
  calibration_raw[0] = readRawSensor();
  Serial.print("Raw value at pH 4.00: ");
  Serial.println(calibration_raw[0], 4);
  
  // Clear any residual data in the serial buffer
  clearSerialBuffer();
  
  // Calibration Point 2: pH 7.0
  Serial.println("2. Immerse the pH sensor in pH 7.00 solution and press any key.");
  waitForUser();
  calibration_raw[1] = readRawSensor();
  Serial.print("Raw value at pH 7.00: ");
  Serial.println(calibration_raw[1], 4);
  
  // Calculate slope and intercept
  slope = (calibration_pH[1] - calibration_pH[0]) / (calibration_raw[1] - calibration_raw[0]);
  intercept = calibration_pH[0] - slope * calibration_raw[0];
  
  // Store calibration data in EEPROM
  EEPROM.put(EEPROM_ADDR_SLOPE, slope);
  EEPROM.put(EEPROM_ADDR_INTERCEPT, intercept);
  
  isCalibrated = true;
  Serial.println("Calibration complete and data saved to EEPROM!");
}

float readRawSensor() {
  // Take multiple readings and average them for stability
  const int numReadings = 10;
  long total = 0;
  for(int i = 0; i < numReadings; i++) {
    total += analogRead(pHSensorPin);
    delay(10); // Small delay between readings
  }
  float average = total / (float)numReadings;
  return average;
}

float readPHValue() {
  float rawSensor = readRawSensor();
  float pH = slope * rawSensor + intercept;
  return pH;
}

void waitForUser() {
  while (Serial.available() == 0) {
    // Wait for user input
  }
  // Read and discard the input
  Serial.read();
}

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any extra spaces or newlines

    if (input.equalsIgnoreCase("cal")) {
      startCalibration();
    }
  }
}
