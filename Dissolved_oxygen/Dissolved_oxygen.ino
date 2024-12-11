#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DO_PIN A6
#define ONE_WIRE_BUS 2    // Pin where the DS18B20 data wire is connected

#define VREF 5000    // VREF (mv)
#define ADC_RES 1024 // ADC Resolution

// Calibration for DO sensor, can be adjusted based on sensor specs
#define V_OFFSET (0)         // Adjust with your sensor's offset value (in mV)
#define V_SLOPE (10.0)       // Adjust with your sensor's slope value (in mV per mg/L)
#define TEMP_COEFFICIENT (0.02) // Temperature correction factor for DO

// Single-point calibration Mode=0
// Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

// Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) // mv
#define CAL1_T (25)   // ℃
// Two-point calibration needs to be filled CAL2_V and CAL2_T
// CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) // mv
#define CAL2_T (15)   // ℃

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

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
  // Calculate the voltage-to-Dissolved Oxygen concentration (mg/L)
  // Adjust this equation based on your sensor's specific calibration
  float DO_concentration = ((voltage_mv - V_OFFSET) / V_SLOPE);
  
  // Apply temperature correction (if necessary)
  float temp_correction = 1 + (TEMP_COEFFICIENT * (temperature_c - 25)); // Correct for 25°C baseline
  DO_concentration *= temp_correction;
  
  // Return the DO concentration as an integer (rounded)
  return (int16_t)DO_concentration;
}

void setup()
{
  Serial.begin(115200);
  sensors.begin();  // Initialize the DS18B20 sensor
}

void loop()
{
  // Request temperature data from the DS18B20 sensor
  sensors.requestTemperatures();
  
  // Read temperature in Celsius from the first sensor (index 0)
  Temperaturet = (uint8_t)sensors.getTempCByIndex(0);
  
  // If the temperature reading is invalid, set a default value
  if (Temperaturet == 85) {  // DS18B20 returns 85 if there's an error
    Temperaturet = 25; // Default to 25°C
  }

  // Read the analog signal from the DO sensor
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;

  // Print out the temperature, ADC raw value, voltage, and DO value in mg/L
  Serial.print("Temperature: " + String(Temperaturet) + "°C\t");
  Serial.print("ADC RAW: " + String(ADC_Raw) + "\t");
  Serial.print("ADC Voltage: " + String(ADC_Voltage) + " mV\t");
  Serial.println("DO: " + String(readDO(ADC_Voltage, Temperaturet)) + " mg/L");

  delay(1000);  // Wait for 1 second before repeating
}
