# Water_Quality_monitoring_Thingspeak_ArduinoMega+ESP8266
 This project used Thingspeak as mqtt broker and getting data through API from a wifi built-in(ESP8266) board of Arduino Mega 2560.

ORP Sensor Calibration Process:

Implements the recommended zero calibration method
Measures reference voltage by shorting S+ and S- pins
Allows user confirmation of calibration
Updates the ORP object's reference voltage dynamically


Calibration Workflow:

Added to the existing performSensorCalibration() function
Provides a comprehensive calibration process for all sensors
Interactive and user-guided


Measurement Integration:

Added ORP value measurement in performSensorOperation()
Included in the data packet for transmission



Calibration Instructions:

Upload the code to your Arduino
Open Serial Monitor
Send 'C' to enter calibration mode
Follow on-screen prompts for each sensor
For ORP sensor:

Disconnect the probe
Short-circuit S+ and S- pins
Confirm the reference voltage when prompted



Important Considerations:

Ensure precise 5V power supply
Use high-quality, stable ADC
Perform calibration in a controlled environment
Short-circuit pins carefully during ORP calibration

Potential Improvements:

Add more robust error handling
Implement advanced calibration techniques
Create a more sophisticated user interface

Thingspeak Dashboard Link: https://thingspeak.mathworks.com/channels/2732596
