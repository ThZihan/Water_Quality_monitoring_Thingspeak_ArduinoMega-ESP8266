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
    if(TurbiditySensorVoltage < 2.5){
      TurbidityInNtu = 3000;
    }else{
      TurbidityInNtu = -1120.4*square(TurbiditySensorVoltage)+5742.3*TurbiditySensorVoltage-4353.8; 
    }
    Serial.print("Turbidity In Ntu: ");
    Serial.println(TurbidityInNtu);
    delay(10);
}
 
float round_to_dpFor_Turb( float in_value, int decimal_place )
{
  float multiplierForTurbidity = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplierForTurbidity ) / multiplierForTurbidity;
  return in_value;
}
