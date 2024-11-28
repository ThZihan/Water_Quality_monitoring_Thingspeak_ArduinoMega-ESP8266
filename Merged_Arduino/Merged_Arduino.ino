#include "GravityTDS.h"
#include "DFRobot_PH.h"
#include "DFRobot_ECPRO.h"
#include "DFRobot_ORP_PRO.h"

// Pin Definitions
#define TDS_PIN A0
#define PH_PIN A1
#define DO_PIN A2
#define EC_PIN A3
#define TE_PIN A4
#define ORP_PIN A5

// Sensor Objects
GravityTDS gravityTds;
DFRobot_PH ph;
DFRobot_ECPRO ec;
DFRobot_ECPRO_PT1000 ecpt;
DFRobot_ORP_PRO ORP(0); // Default reference voltage, to be calibrated

// Global Sensor Variables
float temperature = 25.0;
float tdsValue = 0;
float phValue = 0;
float doValue = 0;
float conductivity = 0;
float orpValue = 0;

// Calibration Mode Flag
bool isCalibrationMode = false;

// ORP Sensor Calibration Function
void calibrateORPSensor() {
  Serial.println("ORP Sensor Calibration");
  Serial.println("====================");
  
  // Zero Calibration Process
  Serial.println("1. Zero Calibration Steps:");
  Serial.println("   - Disconnect ORP probe");
   Serial.println("   - Short-circuit S+ and S- pins");
   Serial.println("   - Measuring reference voltage...");
   
   // Measure reference voltage
   long referenceVoltage = 0;
   int numReadings = 50;
   
   for (int i = 0; i < numReadings; i++) {
     referenceVoltage += ((long)analogRead(ORP_PIN) * 5000 + ADC_RES / 2) / ADC_RES;
     delay(50);
   }
   referenceVoltage /= numReadings;
   
   Serial.print("Measured Reference Voltage: ");
   Serial.print(referenceVoltage);
   Serial.println(" mV");
   
   // Prompt user to confirm calibration
   Serial.println("\nDo you want to use this reference voltage?");
   Serial.println("Enter 'Y' to confirm, 'N' to cancel");
   
   while (!Serial.available()) {
     delay(100);
   }
   
   char response = Serial.read();
   if (response == 'Y' || response == 'y') {
     // Update ORP object with new reference voltage
     ORP = DFRobot_ORP_PRO(referenceVoltage - 2480);
     
     Serial.print("ORP Sensor Calibrated. New Reference Voltage: ");
     Serial.print(ORP.getCalibration());
     Serial.println(" mV");
   } else {
     Serial.println("ORP Calibration Cancelled");
   }
}
// Calibration Function
void performSensorCalibration() {
  Serial.println("Entering Sensor Calibration Mode");
  
  // EC Sensor Calibration
  Serial.println("EC Sensor Calibration:");
  Serial.println("Immerse the probe in the calibration solution");
  Serial.println("Enter 'C + conductivity value' (e.g., C 1413)");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  String args = Serial.readStringUntil('\n');
  if (args[0] == 'C') {
    args.remove(0, 2);
    float calSolution = args.toFloat();
    
    // Measure EC Voltage
    uint16_t EC_Voltage = (uint32_t)analogRead(EC_PIN) * 5000 / 1024;
    
    // Calibrate EC
    float newCalibrationK = ec.calibrate(EC_Voltage, calSolution);
    ec.setCalibration(newCalibrationK);
    
    Serial.print("Calibration Solution: ");
    Serial.println(calSolution);
    Serial.print("New Calibration K: ");
    Serial.println(newCalibrationK, 6);
  }
  
  // pH Sensor Calibration
  Serial.println("\npH Sensor Calibration:");
  Serial.println("Use standard pH buffer solutions (4.0, 7.0, 10.0)");
  Serial.println("Enter 'P + pH value' (e.g., P 7.0)");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  args = Serial.readStringUntil('\n');
  if (args[0] == 'P') {
    args.remove(0, 2);
    float calPH = args.toFloat();
    
    // Measure pH Voltage
    float voltage = analogRead(PH_PIN) / 1024.0 * 5000;
    
    // Calibrate pH (you may need to implement a specific calibration method)
    ph.calibration(voltage, temperature);
    
    Serial.print("Calibration pH: ");
    Serial.println(calPH);
  }
  
  // DO Sensor Calibration (Simple Single-Point Calibration)
  Serial.println("\nDO Sensor Calibration:");
  Serial.println("Ensure sensor is in saturated water");
  Serial.println("Press any key to continue");
  
  while (!Serial.available()) {
    delay(100);
  }
  
  // Perform DO calibration measurement
  uint16_t ADC_Raw = analogRead(DO_PIN);
  uint16_t ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  
  Serial.print("DO Calibration Voltage: ");
  Serial.println(ADC_Voltage);

 // Add ORP Sensor Calibration
  Serial.println("\nORP Sensor Calibration:");
  calibrateORPSensor();
  
  Serial.println("Calibration Complete. Switching to Operation Mode.");
  isCalibrationMode = false;
}

// Operation Function for Normal Sensor Reading
void performSensorOperation() {
  // Temperature Measurement
  uint16_t TE_Voltage = (uint32_t)analogRead(TE_PIN) * 5000 / 1024;
  temperature = ecpt.convVoltagetoTemperature_C((float)TE_Voltage/1000);
  
  // TDS Measurement
  gravityTds.setTemperature(temperature);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
  
  // pH Measurement
  float voltage = analogRead(PH_PIN) / 1024.0 * 5000;
  phValue = ph.readPH(voltage, temperature);
  
  // DO Measurement
  uint16_t ADC_Raw = analogRead(DO_PIN);
  uint16_t ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  doValue = readDO(ADC_Voltage, (uint8_t)temperature);
  
  // EC Measurement
  uint16_t EC_Voltage = (uint32_t)analogRead(EC_PIN) * 5000 / 1024;
  conductivity = ec.getEC_us_cm(EC_Voltage, temperature);

    // ORP Measurement
  unsigned int ADC_voltage = ((unsigned long)analogRead(ORP_PIN) * 5000 + 1024 / 2) / 1024;
  orpValue = ORP.getORP(ADC_voltage);
  
 // Create Data Packet (include ORP value)
  String dataPacket = "T:" + String(temperature, 2) +
                      "|P:" + String(phValue, 2) +
                      "|TD:" + String(tdsValue, 0) +
                      "|D:" + String(doValue, 2) +
                      "|E:" + String(conductivity, 2) +
                      "|O:" + String(orpValue, 2) + // Add ORP value
                      "\n";
  
  // Send Data
  Serial3.print(dataPacket);
  Serial.println(dataPacket);
}

// DO Sensor Reading Function (from original DO code)
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
  #if TWO_POINT_CALIBRATION == 0
    uint16_t V_saturation = (uint32_t)131 + (uint32_t)35 * temperature_c - (uint32_t)25 * 35;
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
  #else
    uint16_t V_saturation = (int16_t)((int8_t)temperature_c - 15) * ((uint16_t)131 - 1300) / (25 - 15) + 1300;
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
  #endif
}

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  
  // Sensor Initializations
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(5.0);
  gravityTds.setAdcRange(1024);
  gravityTds.begin();
  
  ph.begin();
  
  ec.setCalibration(1);
}

void loop() {
  // Check for calibration command
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'C') {
      isCalibrationMode = true;
    }
  }
  
  // Run either calibration or operation mode
  if (isCalibrationMode) {
    performSensorCalibration();
  } else {
    performSensorOperation();
  }
  
  delay(1000);
}

// DO Sensor Lookup Table (from original DO code)
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};
