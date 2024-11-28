#include "GravityTDS.h"
#include "DFRobot_PH.h"
#include "DFRobot_ECPRO.h"

// Pin Definitions
#define TDS_PIN A0
#define PH_PIN A1
#define EC_PIN A2
#define TE_PIN A3

// Sensor Objects
GravityTDS gravityTds;
DFRobot_PH ph;
DFRobot_ECPRO ec;
DFRobot_ECPRO_PT1000 ecpt;

// Global Sensor Variables
float temperature = 25.0;  // Will be replaced by actual temperature reading
float tdsValue = 0;
float phValue = 0;
float voltage = 0;
uint16_t EC_Voltage, TE_Voltage;
float Conductivity;

void setup() {
  Serial.begin(9600);         // Debugging
  Serial3.begin(9600);        // Communication with ESP8266 at a stable baud rate
  
  // TDS Sensor Initialization
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(5.0);    // reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  // 1024 for 10bit ADC; 4096 for 12bit ADC
  gravityTds.begin();  // initialization

  // pH Sensor Initialization
  ph.begin();

  // EC Sensor Initialization
  ec.setCalibration(1); // Replace with calibrated K value if calibrated
  Serial.println("Default Calibration K=" + String(ec.getCalibration()));
}

void loop() {
  static unsigned long timepoint = millis();
  
  // Generate random values for some sensors
  float DO = random(85, 90)/ 10.0;
  float ORP = random(36000, 45000)/ 100.0;
  int turbidity = random(109, 200);        
      
  // Temperature Measurement
  TE_Voltage = (uint32_t)analogRead(TE_PIN) * 5000 / 1024;
  temperature = ecpt.convVoltagetoTemperature_C((float)TE_Voltage/1000);
  
  // TDS Sensor Measurement
  gravityTds.setTemperature(temperature);  // set the temperature for compensation
  gravityTds.update();  // sample and calculate
  tdsValue = gravityTds.getTdsValue();  // get the TDS value

  // pH Sensor Measurement
  if(millis() - timepoint > 1000U){  // time interval: 1s
    timepoint = millis();
    
    // Read voltage and convert to pH with temperature compensation
    voltage = analogRead(PH_PIN) / 1024.0 * 5000;  // read the voltage
    phValue = ph.readPH(voltage, temperature);  // convert voltage to pH
  }

  // EC Sensor Measurement
  EC_Voltage = (uint32_t)analogRead(EC_PIN) * 5000 / 1024;
  Conductivity = ec.getEC_us_cm(EC_Voltage, temperature);
  
  // Optional: Uncomment for calibration if needed
  // ph.calibration(voltage, temperature);
  
  // Create formatted string with delimiters
  String dataPacket = "T:" + String(temperature, 2) +  // Temperature with 2 decimal places
                      "|P:" + String(phValue, 2) +     // pH with 2 decimal places
                      "|TD:" + String(tdsValue, 0) +   // TDS with 0 decimal places
                      "|TU:" + String(turbidity) +
                      "|D:" + String(DO) +
                      "|O:" + String(ORP) +
                      "|E:" + String(Conductivity, 2) +  // Conductivity with 2 decimal places
                      "\n";
  
  // Send data packet to ESP8266
  Serial3.print(dataPacket);
  
  // Debugging output
  Serial.println(dataPacket);
  
  // Wait 1 second before sending the next packet
  delay(1000);
}
