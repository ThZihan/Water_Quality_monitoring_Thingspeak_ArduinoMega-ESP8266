void setup() {
  Serial.begin(9600);         // Debugging
  Serial3.begin(9600);        // Communication with ESP8266 at a stable baud rate
}

void loop() {
  // Generate random values for sensors
  float temperature = random(200, 260) / 10.0;  
  float pH = random(60, 80)/ 10.0;             
  int tds = random(20, 40);
  int turbidity = random(109, 200);        
  
  // Create formatted string with delimiters
  String dataPacket = "T:" + String(temperature) +
                      "|P:" + String(pH) +
                      "|TDS:" + String(tds) +
                      "|TUR:" + String(turbidity)+ "\n";

  // Send data packet to ESP8266
  Serial3.print(dataPacket);

  // Debugging output
  Serial.println(dataPacket);

  // Wait 1 second before sending the next packet
  delay(1000);
}
