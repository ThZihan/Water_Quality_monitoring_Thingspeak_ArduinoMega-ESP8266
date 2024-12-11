#include "DFRobot_ECPRO.h"

#define EC_PIN A0
#define TE_PIN A3

DFRobot_ECPRO ec;
DFRobot_ECPRO_PT1000 ecpt;

uint16_t EC_Voltage, TE_Voltage;
float Conductivity, Temp;

void setup()
{
  Serial.begin(115200);

  ec.setCalibration(1);
  Serial.println("Default Calibration K=" + String(ec.getCalibration()));
  Serial.println("Immerse the probe in the calibration solution and enter \"C + conductivity\" to calibrate");
  Serial.println("e.g. \"C 1413\"");

  delay(2000);
}

void loop()
{
  EC_Voltage = (uint32_t)analogRead(EC_PIN) * 5000 / 1024;
  //TE_Voltage = (uint32_t)analogRead(TE_PIN) * 5000 / 1024;
  TE_Voltage = 783;

  Temp = ecpt.convVoltagetoTemperature_C((float)TE_Voltage / 1000);
  Conductivity = ec.getEC_us_cm(EC_Voltage, Temp);

  Serial.print("EC_Voltage: " + String(EC_Voltage) + " mV\t");
  Serial.print("Conductivity: " + String(Conductivity) + " us/cm\t");
  Serial.print("TE_Voltage: " + String(TE_Voltage) + " mV\t");
  Serial.println("Temp: " + String(Temp) + " ℃");

  delay(1000);
}

void serialEvent()
{
  String args = Serial.readStringUntil('\n');
  if (args[0] == 'C')
  {
    args.remove(0, 2);
    float calSoluion=args.toFloat();
    ec.setCalibration(ec.calibrate(EC_Voltage,calSoluion));
    Serial.print("calSoluion: " + String(calSoluion)+"\t");
    Serial.println("calK: "+ String(ec.getCalibration(),6));
    delay(5000);
  }
  else
    Serial.println("command error");
}
