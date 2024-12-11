#include <Wire.h> 

const int pHSensorPin = A4; // Pin connected to the pH sensor's analog output
float calibration_pH[2] = {4.0, 7.0};     // Known pH values for calibration
float calibration_raw[2] = {0.0, 0.0};    // Raw sensor values for calibration
bool isCalibrated = false;

void setup() {
  // Start serial communication
  Serial.begin(9600);
  while (!Serial) { ; } // Wait for serial port to connect. Needed for native USB
  Serial.println("DFRobot Gravity pH Meter Pro V2 Calibration and Readout");
  Serial.println("Connecting to pH sensor...");
  
  delay(2000);
  Serial.println("Ready to read pH value or calibrate.");
  Serial.println("Enter 'cal' to start calibration or press any key to read pH value.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any extra spaces or newlines

    if (input.equalsIgnoreCase("cal")) {
      startCalibration();
    } else {
      readPHValue();
    }
  }
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
  
  isCalibrated = true;
  Serial.println("Calibration complete!");
  Serial.println("You can now start reading pH values.");
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

float calculatePH(float rawValue) {
  if (!isCalibrated) {
    Serial.println("Sensor not calibrated. Please calibrate first by entering 'cal'.");
    return 0.0;
  }

  // Calculate slope and intercept based on calibration data
  float slope = (calibration_pH[1] - calibration_pH[0]) / (calibration_raw[1] - calibration_raw[0]);
  float intercept = calibration_pH[0] - slope * calibration_raw[0];
  
  // Calculate pH
  float pH = slope * rawValue + intercept;
  return pH;
}

void readPHValue() {
  float rawSensor = readRawSensor();
  float pH = calculatePH(rawSensor);
  
  Serial.print("Raw Sensor Value: ");
  Serial.print(rawSensor, 4);
  Serial.print(" | Current pH: ");
  Serial.println(pH, 2);
  
  delay(1000); // Delay between readings
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
