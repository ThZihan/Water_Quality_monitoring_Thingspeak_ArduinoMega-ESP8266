#include <Wire.h>
#include "DFRobot_ORP_PRO.h"
#include "DFRobot_ECPRO.h"
#include <OneWire.h>
#include <DallasTemperature.h>

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
  Serial.begin(115200);       // ORP sensor communication
  Serial.print("calibration is: ");
  Serial.print(ORP.getCalibration());
  Serial.println(" mV");

  // Initialize the temperature sensor
  sensors.begin();
}

void loop() {
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
  Serial.print("ORP value is: ");
  Serial.print(ORP.getORP(ADC_voltage));
  Serial.println(" mV");

  // Read the EC sensor
  EC_Voltage = (uint32_t)analogRead(EC_PIN) * 5000 / 1024;
  TE_Voltage = (uint32_t)analogRead(TE_PIN) * 5000 / 1024;

  Temp = ecpt.convVoltagetoTemperature_C((float)TE_Voltage / 1000);
  Conductivity = ec.getEC_us_cm(EC_Voltage, Temp);

  // Read the temperature and DO sensor
  sensors.requestTemperatures(); // Get temperature from DS18B20
  float realTemperature = sensors.getTempCByIndex(0);  // Temperature in Celsius
  Serial.print("Temperature: ");
  Serial.print(realTemperature);
  Serial.print(" °C\t");

  // Read the DO sensor
  DO = analogRead(DO_PIN);
  float DO_voltage = ((float)DO * V_REF) / ADC_RES;
  int DO_concentration = readDO(DO_voltage, realTemperature);  // Calculate DO concentration
  
  Serial.print("DO: ");
  Serial.print(DO_concentration);
  Serial.println(" mg/L");

  // Output data
  String dataPacket = "T:" + String(realTemperature) +
                      "|P:" + String(tdsValue) +
                      "|TD:" + String(tdsValue) +
                      "|TU:" + String(TurbidityInNtu) +
                      "|D:" + String(DO_concentration) +
                      "|O:" + String(ORP.getORP(ADC_voltage)) +
                      "|E:" + String(Conductivity) +
                      "\n";

  // Send data packet to ESP8266
  Serial3.print(dataPacket);

  // Debugging output
  Serial.println(dataPacket);

  // Wait 1 second before sending the next packet
  delay(1000);
}

// Function to calculate Dissolved Oxygen
int readDO(uint32_t voltage_mv, uint8_t temperature_c) {
  float DO_concentration = ((voltage_mv - V_OFFSET) / V_SLOPE);
  float temp_correction = 1 + (TEMP_COEFFICIENT * (temperature_c - 25));  // Adjust based on temperature
  return int(DO_concentration * temp_correction);
}
