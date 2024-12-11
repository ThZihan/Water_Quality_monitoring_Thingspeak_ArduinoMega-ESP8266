#include <Wire.h>

const int pHSensorPin = A0; // Pin connected to the pH sensor's analog output
float pHValue = 0.0;         // Variable to store pH value
float calibrationBuffer[2];  // Array to store calibration values (low and high points)

void setup() {
  // Start serial communication
  Serial.begin(9600);
  Serial.println("DFRobot Gravity pH Meter Pro V2 - Data Logging");
  Serial.println("Connecting to pH sensor...");

  // Default calibration values (adjust after calibration)
  calibrationBuffer[0] = 4.0;  // pH 4.0 calibration value
  calibrationBuffer[1] = 7.0;  // pH 7.0 calibration value

  delay(2000);
  Serial.println("Ready to read pH value.");
  Serial.println("Reading pH...");
}

void loop() {
  // Read and convert raw sensor value
  float rawSensorValue = readRawPH();

  // Convert raw sensor value to pH using calibration values
  pHValue = convertToPH(rawSensorValue);

  // Print pH value to Serial Monitor
  Serial.print("Current pH: ");
  Serial.println(pHValue, 2);

  delay(1000);  // Delay for 1 second before reading again
}

float readRawPH() {
  int rawValue = analogRead(pHSensorPin);
  float voltage = rawValue * (5.0 / 1023.0);  // Convert raw value to voltage (assuming 5V reference)
  return voltage;
}

float convertToPH(float rawVoltage) {
  // Assume a linear relationship between voltage and pH value (this needs to be adjusted according to your sensor's characteristics)
  float slope = (calibrationBuffer[1] - calibrationBuffer[0]) / (5.0 - 0.0);  // Slope calculated from known calibration values
  float intercept = calibrationBuffer[0] - slope * 0.0;  // Intercept calculated using pH 0.0 at 0.0V
  
  // Calculate pH based on voltage and calibration values
  float pH = slope * rawVoltage + intercept;
  return pH;
}
