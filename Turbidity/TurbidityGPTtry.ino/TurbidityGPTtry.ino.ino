#include <Wire.h> 

int TurbiditySensorPin = A5;
float TurbiditySensorVoltage;
float TurbidityInNtu;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
    TurbiditySensorVoltage = 0;
    for(int i=0; i<800; i++)
    {
        TurbiditySensorVoltage += ((float)analogRead(TurbiditySensorPin)/1023)*5;
    }
    TurbiditySensorVoltage = TurbiditySensorVoltage/800;
    TurbiditySensorVoltage = round_to_dpFor_Turb(TurbiditySensorVoltage,2);
    
    // Replace with your calibrated formula
    TurbidityInNtu = 10.0 * TurbiditySensorVoltage - 20.0;
    
    // Apply warning and alarm thresholds based on calibrated NTU
    if(TurbidityInNtu > 30){
        // Trigger alarm
    } else if(TurbidityInNtu > 10){
        // Trigger warning
    }
    
    Serial.print("Turbidity In NTU: ");
    Serial.println(TurbidityInNtu);
    delay(1000); // Adjust delay as needed
}

float round_to_dpFor_Turb(float in_value, int decimal_place)
{
  float multiplierForTurbidity = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplierForTurbidity) / multiplierForTurbidity;
  return in_value;
}
