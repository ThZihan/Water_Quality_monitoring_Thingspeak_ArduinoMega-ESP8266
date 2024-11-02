#include <ESP8266WiFi.h>
#include <WiFiClient.h>

const char* ssid = "MasterShifu";
const char* password = "Tweety@Birdy";
const char* apiKey = "27DD5GVDLU2XBYDT";  // Write API Key
const char* server = "api.thingspeak.com";
int interval = 15000; // Data sending interval in ms

WiFiClient client;
String dataBuffer = "";  // Buffer to store incoming data from Arduino

void setup() {
  Serial.begin(9600);  // Communication with Arduino Mega
  Serial.println("ESP8266 is ready");
  connectToWiFi();
}

void loop() {
  // Ensure WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  // Read data from Arduino Mega
  while (Serial.available()) {
    char c = Serial.read();
    dataBuffer += c;

    // Process and send data if we detect the end of a packet
    if (c == '\n') {
      if (processAndSendData(dataBuffer)) {
        Serial.println("Data sent to ThingSpeak successfully.");
      } else {
        Serial.println("Error: Invalid data format.");
      }
      dataBuffer = "";  // Clear buffer after processing
    }
  }

  delay(100);  // Short delay to allow data transmission
}

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");

  WiFi.begin(ssid, password);
  int attempt = 0;

  // Wait for connection with a maximum of 10 attempts
  while (WiFi.status() != WL_CONNECTED && attempt < 10) {
    delay(500);
    Serial.print(".");
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi");
  } else {
    Serial.println("\nFailed to connect to Wi-Fi. Retrying...");
    delay(5000);  // Wait before retrying
    connectToWiFi();
  }
}

bool processAndSendData(String data) {
  float Temperature, pH;
  int Tds, Turbidity;

  // Parse the data received from Arduino
  int result = sscanf(data.c_str(), "T:%f|P:%f|TDS:%d|TUR:%d", &Temperature, &pH, &Tds, &Turbidity);

  // Check if parsing was successful
  if (result != 4) {
    return false;  // Return false if parsing fails
  }

  // Send data to ThingSpeak
  if (client.connect(server, 80)) {
    String postStr = apiKey;
    postStr += "&field1="; postStr += String(Temperature);
    postStr += "&field2="; postStr += String(pH);
    postStr += "&field3="; postStr += String(Tds);
    postStr += "&field4="; postStr += String(Turbidity);
    postStr += "\r\n\r\n";

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + String(apiKey) + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: " + String(postStr.length()) + "\n\n");
    client.print(postStr);

    // Wait for server response
    delay(1000);  // Give some time for response
    if (client.available()) {
      String response = client.readString();
      Serial.print("ThingSpeak Response: ");
      Serial.println(response);
    } else {
      Serial.println("No response from ThingSpeak");
    }

    client.stop();  // Close the connection
    return true;  // Data was sent successfully
  } else {
    Serial.println("Connection to ThingSpeak failed");
    return false;
  }
}
