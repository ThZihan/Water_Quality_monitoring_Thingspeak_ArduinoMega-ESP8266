#include "DFRobot_ORP_PRO.h"

#define PIN_ORP A1
#define ADC_RES 1024
#define V_REF 5000

unsigned int ADC_voltage;

DFRobot_ORP_PRO ORP(-9);//set reference voltage mv

void setup() {
  Serial.begin(115200);

  Serial.print("calibration is: ");
  Serial.print(ORP.getCalibration());
  Serial.println(" mV");
}

void loop() {
  ADC_voltage = ((unsigned long)analogRead(PIN_ORP) * V_REF + ADC_RES / 2) / ADC_RES;
  Serial.print("ORP value is: ");
  Serial.print(ORP.getORP(ADC_voltage));
  Serial.println(" mV");
  delay(1000);
}
