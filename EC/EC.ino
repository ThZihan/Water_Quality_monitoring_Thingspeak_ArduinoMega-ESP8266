#include "DFRobot_ECPRO.h"

#define EC_PIN A8
#define TE_PIN A7

DFRobot_ECPRO ec;
DFRobot_ECPRO_PT1000 ecpt;

uint16_t EC_Voltage, TE_Voltage;
float Conductivity, Temp;

void setup()
{
  Serial.begin(115200);

  ec.setCalibration(1); //Replace the 1 with the calibrated K value if it's calibrated
  Serial.println("Default Calibration K=" + String(ec.getCalibration()));
}

void loop()
{
  EC_Voltage = (uint32_t)analogRead(EC_PIN) * 5000 / 1024;
  TE_Voltage = (uint32_t)analogRead(TE_PIN) * 5000 / 1024;

  Temp = ecpt.convVoltagetoTemperature_C((float)TE_Voltage/1000);
  Conductivity = ec.getEC_us_cm(EC_Voltage, Temp);

  Serial.print("Conductivity: " + String(Conductivity) + " us/cm\t");
  Serial.println("Temp: " + String(Temp) + " ℃");

  delay(1000);
}
