#define PIN_ORP A1
#define ADC_RES 1024
#define V_REF 5000

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  Serial.println((((long)analogRead(PIN_ORP)*V_REF + ADC_RES / 2) / ADC_RES) - 2480);
  delay(1000);
}
