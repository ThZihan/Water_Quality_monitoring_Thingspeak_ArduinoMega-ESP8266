#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// GPIO Definitions
const int GREEN_LED = 5; // GPIO5
const int RED_LED = 4; // GPIO4

// Wi-Fi Credentials
const char* ssid = "R A S....... SAU.";
const char* password = "11223344ras";

// ThingSpeak Configuration
const char* apiKey = "7LNV7Z12NVHQIGW6";  // Your ThingSpeak API Key
const char* server = "api.thingspeak.com";
const int port = 80; // HTTP port for ThingSpeak

// Data Sending Interval
const unsigned long DATA_INTERVAL = 15000; // 15 seconds

// Wi-Fi Connection Timeout
const unsigned long WIFI_TIMEOUT = 60000; // 1 minute in milliseconds

WiFiClient client;
String dataBuffer = "";  // Buffer to store incoming data from Arduino

// Timer Variables
unsigned long previousDataSendTime = 0;
unsigned long wifiConnectStartTime = 0;
bool isReconnecting = false;

void setup() {
  Serial.begin(9600);  // Communication with Arduino Mega
  Serial.println("ESP8266 is initializing...");

  // Initialize GPIOs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  
  // Initially, assume Wi-Fi is not connected
  digitalWrite(GREEN_LED, HIGH); // Indicate Wi-Fi not connected
  digitalWrite(RED_LED, LOW);  // Ensure GPIO5 is LOW
  
  connectToWiFi();
}

void loop() {
  // Check Wi-Fi connection status
  if (WiFi.status() != WL_CONNECTED) {
    // If not connected, ensure GPIO4 is HIGH and GPIO5 is LOW
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    
    // Start reconnection process if not already attempting
    if (!isReconnecting) {
      Serial.println("Wi-Fi disconnected. Starting reconnection attempts...");
      isReconnecting = true;
      wifiConnectStartTime = millis();
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }

    // Check if the connection attempt has timed out
    if (millis() - wifiConnectStartTime >= WIFI_TIMEOUT) {
      Serial.println("Wi-Fi connection timeout. Restarting ESP8266...");
      ESP.restart(); // Restart the ESP8266
    }
  } else {
    // If connected, ensure GPIO4 is LOW and GPIO5 is HIGH
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    isReconnecting = false; // Reset reconnection flag

    // Handle data sending at specified intervals
    if (millis() - previousDataSendTime >= DATA_INTERVAL) {
      previousDataSendTime = millis();
      // You can place data sending functions here if needed periodically
    }
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
        Serial.println("Error: Invalid data format or failed to send.");
      }
      dataBuffer = "";  // Clear buffer after processing
    }
  }

  delay(100);  // Short delay to allow data transmission
}

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  wifiConnectStartTime = millis(); // Start the timeout timer

  // Attempt to connect for up to WIFI_TIMEOUT duration
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
    // Check if timeout has been exceeded
    if (millis() - wifiConnectStartTime >= WIFI_TIMEOUT) {
      Serial.println("\nFailed to connect to Wi-Fi within timeout.");
      Serial.println("Restarting ESP8266...");
      ESP.restart();
    }
  }

  // Connected to Wi-Fi
  Serial.println("\nConnected to Wi-Fi");
  digitalWrite(GREEN_LED, LOW);  // Indicate Wi-Fi is connected
  digitalWrite(RED_LED, HIGH); // Indicate GPIO5 is HIGH
}

bool processAndSendData(String data) {
  float Temperature, pH, DO, ORP;
  int Tds, Turbidity, EC;

  // Parse the data received from Arduino
  int result = sscanf(data.c_str(), "T:%f|P:%f|TD:%d|TU:%d|D:%f|O:%f|E:%d", 
                      &Temperature, &pH, &Tds, &Turbidity, &DO, &ORP, &EC);

  // Check if parsing was successful
  if (result != 7) {
    Serial.println("Data parsing failed.");
    return false;  // Return false if parsing fails
  }

  // Prepare the POST data
  String postData = "api_key=" + String(apiKey);
  postData += "&field1=" + String(pH);
  postData += "&field2=" + String(Turbidity);
  postData += "&field3=" + String(DO);
  postData += "&field4=" + String(EC);
  postData += "&field5=" + String(Temperature);
  postData += "&field6=" + String(ORP);
  postData += "&field7=" + String(Tds);

  // Connect to ThingSpeak
  if (client.connect(server, port)) {
    Serial.println("Connected to ThingSpeak. Sending data...");

    // Construct HTTP POST request
    client.println("POST /update HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Connection: close");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(postData.length());
    client.println();
    client.println(postData);

    // Wait for server response
    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) { // 5 seconds timeout
        Serial.println(">>> Client Timeout !");
        client.stop();
        return false;
      }
    }

    // Read response
    String response = client.readString();
    Serial.println("ThingSpeak Response:");
    Serial.println(response);

    client.stop();  // Close the connection
    return true;    // Data was sent successfully
  } else {
    Serial.println("Connection to ThingSpeak failed.");
    return false;
  }
}
