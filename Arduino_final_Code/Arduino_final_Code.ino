#include <Wire.h>
#include "DFRobot_ORP_PRO.h"
#include "DFRobot_ECPRO.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

// Pin and constants for TDS sensor
#define TdsSensorPin A1
#define VREF 5.0              // Analog reference voltage (Volt) of the ADC
#define SCOUNT  30            // Sum of sample points
int analogBuffer[SCOUNT];     // Store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 16;       // Current temperature for compensation

// Pin and constants for turbidity sensor
int TurbiditySensorPin = A5;
float TurbiditySensorVoltage;
float TurbidityInNtu;

// Pin and constants for ORP sensor
#define PIN_ORP A2
#define ADC_RES 1024
#define V_REF 5000
unsigned int ADC_voltage;
DFRobot_ORP_PRO ORP(-14);  // Set reference voltage (mV)

// Pin and constants for EC sensor
#define EC_PIN A0
#define TE_PIN A3
const int pHSensorPin = A4;
DFRobot_ECPRO ec;
DFRobot_ECPRO_PT1000 ecpt;

uint16_t EC_Voltage, TE_Voltage;
float Conductivity, Temp;

// Pin and constants for DO and temperature sensors
#define DO_PIN A6
#define ONE_WIRE_BUS 2    // Pin where the DS18B20 data wire is connected

// Calibration for DO sensor
#define V_OFFSET (0)         // Adjust with your sensor's offset value (in mV)
#define V_SLOPE (10.0)       // Adjust with your sensor's slope value (in mV per mg/L)
#define TEMP_COEFFICIENT (0.02) // Temperature correction factor for DO

// Single-point calibration Mode=0
// Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

// Single point calibration values
#define CAL1_V (1600) // mv
#define CAL1_T (25)   // ℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

// Create an instance of the OneWire and DallasTemperature objects
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Calibration data for pH
float calibration_pH[2] = {4.0, 7.0};     // Known pH values for calibration
float calibration_pHraw[2] = {0.0, 0.0};    // Raw sensor values for calibration
bool pHisCalibrated = false;
float pH;
float pHslope = 0.0;
float pHintercept = 0.0;
// EEPROM addresses
#define EEPROM_ADDR_SLOPE 0
#define EEPROM_ADDR_INTERCEPT 4

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

// Median filtering algorithm for TDS sensor
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  return bTab[iFilterLen / 2];  // Return the median value
}

// Function to round the turbidity value to 2 decimal places
float round_to_dpFor_Turb(float in_value, int decimal_place) {
  float multiplierForTurbidity = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplierForTurbidity) / multiplierForTurbidity;
  return in_value;
}

void setup() {
  Serial.begin(9600);         // Debugging
  Serial3.begin(9600);        // Communication with ESP8266 at a stable baud rate
  eepromCheckpH();
  // Initialize the temperature sensor
  sensors.begin();
  
}

void loop() {
  handleSerialInput(); // Check for calibration command for pH
  if (pHisCalibrated) {
    pH = readPHValue();
  } else {
    Serial.println("Sensor not calibrated. Please calibrate first by entering 'cal'.");
  }
  
  // Read the TDS sensor
  for (int i = 0; i < SCOUNT; i++) {
    analogBuffer[i] = analogRead(TdsSensorPin);
    delay(10); // Give some time between readings
  }

  // Apply median filter to the sensor readings
  tdsValue = getMedianNum(analogBuffer, SCOUNT);

  // Convert the raw sensor value to TDS (assuming 5V reference voltage)
  averageVoltage = tdsValue * VREF / 1024.0;
  tdsValue = (133.42 * pow(averageVoltage, 3) - 255.86 * pow(averageVoltage, 2) + 857.39 * averageVoltage) * 0.5;  // Conversion to TDS value

  // Read the turbidity sensor
  TurbiditySensorVoltage = 0;
  for (int i = 0; i < 800; i++) {
    TurbiditySensorVoltage += ((float)analogRead(TurbiditySensorPin) / 1023) * 5;
  }
  TurbiditySensorVoltage = TurbiditySensorVoltage / 800;
  TurbiditySensorVoltage = round_to_dpFor_Turb(TurbiditySensorVoltage, 2);
  
  if (TurbiditySensorVoltage < 2.5) {
    TurbidityInNtu = 3000;
  } else {
    TurbidityInNtu = -1120.4 * square(TurbiditySensorVoltage) + 5742.3 * TurbiditySensorVoltage - 4353.8;
  }

  // Read the ORP sensor
  ADC_voltage = ((unsigned long)analogRead(PIN_ORP) * V_REF + ADC_RES / 2) / ADC_RES;
 
  // Read the EC sensor
  EC_Voltage = (uint32_t)analogRead(EC_PIN) * 5000 / 1024;
  TE_Voltage = (uint32_t)analogRead(TE_PIN) * 5000 / 1024;

  Temp = ecpt.convVoltagetoTemperature_C((float)TE_Voltage / 1000);
  Conductivity = ec.getEC_us_cm(EC_Voltage, Temp);

  // Read the temperature and DO sensor
  sensors.requestTemperatures(); // Get temperature from DS18B20
  float realTemperature = sensors.getTempCByIndex(0);  // Temperature in Celsius

  // Read the DO sensor
  DO = analogRead(DO_PIN);
  float DO_voltage = ((float)DO * V_REF) / ADC_RES;
  int DO_concentration = readDO(DO_voltage, realTemperature);  // Calculate DO concentration
 

  // Output data
  String dataPacket = "T:" + String(realTemperature) +
                      "|P:" + String(pH) +
                      "|TD:" + String(int(tdsValue)) +
                      "|TU:" + String(int(TurbidityInNtu)) +
                      "|D:" + String(float(DO_concentration)) +
                      "|O:" + String(ORP.getORP(ADC_voltage)) +
                      "|E:" + String(int(Conductivity)) +
                      "\n";

  // Debugging output
  Serial3.println(dataPacket);
  Serial.println(dataPacket);

  // Wait 1 second before sending the next packet
  delay(1000);
}
void eepromCheckpH(){
    // Load calibration data from EEPROM
  EEPROM.get(EEPROM_ADDR_SLOPE, pHslope);
  EEPROM.get(EEPROM_ADDR_INTERCEPT, pHintercept);
  
  if (pHslope != 0.0 && pHintercept != 0.0) {
    pHisCalibrated = true;
    Serial.println("Calibration data loaded from EEPROM.");
  } else {
    Serial.println("No calibration data found. Please calibrate by entering 'cal'.");
  }
  
  Serial.println("Enter 'cal' to start calibration.");
}
// Function to calculate Dissolved Oxygen
int readDO(uint32_t voltage_mv, uint8_t temperature_c) {
  float DO_concentration = ((voltage_mv - V_OFFSET) / V_SLOPE);
  float temp_correction = 1 + (TEMP_COEFFICIENT * (temperature_c - 25));  // Adjust based on temperature
  return int(DO_concentration * temp_correction);
}

void startCalibration() {
  Serial.println("Starting calibration process...");
  
  // Calibration Point 1: pH 4.0
  Serial.println("1. Immerse the pH sensor in pH 4.00 solution and press any key.");
  waitForUser();
  calibration_pHraw[0] = readRawSensor();
  Serial.print("Raw value at pH 4.00: ");
  Serial.println(calibration_pHraw[0], 4);
  
  // Clear any residual data in the serial buffer
  clearSerialBuffer();
  
  // Calibration Point 2: pH 7.0
  Serial.println("2. Immerse the pH sensor in pH 7.00 solution and press any key.");
  waitForUser();
  calibration_pHraw[1] = readRawSensor();
  Serial.print("Raw value at pH 7.00: ");
  Serial.println(calibration_pHraw[1], 4);
  
  // Calculate slope and intercept
  pHslope = (calibration_pH[1] - calibration_pH[0]) / (calibration_pHraw[1] - calibration_pHraw[0]);
  pHintercept = calibration_pH[0] - pHslope * calibration_pHraw[0];
  
  // Store calibration data in EEPROM
  EEPROM.put(EEPROM_ADDR_SLOPE, pHslope);
  EEPROM.put(EEPROM_ADDR_INTERCEPT, pHintercept);
  
  pHisCalibrated = true;
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
  float pH = pHslope * rawSensor + pHintercept;
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
