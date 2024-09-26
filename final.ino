#include <Wire.h>
#include <WiFi.h>  // ESP32 or ESP8266 Wi-Fi library
#include "ThingSpeak.h"
#include "MPU6050.h"
#include <PubSubClient.h>  // MQTT client for ThingsBoard
// SDA 21   SCL 22   ADO of gyro 1 is connected ground and ADO vcc
// Wi-Fi credentials
const char* ssid = "Galaxy S";
const char* password = "Frequency";

// ThingSpeak channel details
unsigned long myChannelNumber = 2670907;
const char* myWriteAPIKey = "XLF44CFYIW3316Y2";

// ThingSpeak client
WiFiClient client;

// ThingsBoard credentials
const char* mqttServer = "demo.thingsboard.io"; // Replace with your ThingsBoard server
const int mqttPort = 1883;
const char* thingsboardToken = "gchq6ion4hrrc27q1cow"; // Replace with your device's access token

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// MPU6050 setup for two sensors
MPU6050 mpu1(0x68);  // First MPU6050 on default I2C address
MPU6050 mpu2(0x69);  // Second MPU6050 on alternate I2C address

// Sensor pins
int emgPin = 32;        // EMG sensor connected to analog pin A0
int fsrHeelPin = 35;    // FSR sensor (heel) connected to analog pin A1
int fsrToePin = 34;     // FSR sensor (toe) connected to analog pin A2

// Thresholds for FSR sensor to detect gait phases
int fsrThresholdHeel = 300;  // Adjust based on your calibration
int fsrThresholdToe = 300;   // Adjust based on your calibration

// Function to determine the gait phase
String getGaitPhase(int heelPressure, int toePressure) {
  if (heelPressure > fsrThresholdHeel && toePressure > fsrThresholdToe) {
    return "Midstance";
  } else if (heelPressure > fsrThresholdHeel && toePressure < fsrThresholdToe) {
    return "Heel Strike";
  } else if (heelPressure < fsrThresholdHeel && toePressure > fsrThresholdToe) {
    return "Toe Off";
  } else {
    return "Swing";
  }
}

// Function to determine motion state based on gyroscope data
String getMotionState(int16_t gy1, int16_t gy2) {
  float gyroThreshold = 2000; // Threshold for motion detection (adjust as necessary)
  
  if (abs(gy1) < gyroThreshold && abs(gy2) < gyroThreshold) {
    return "Sitting";
  } else if (abs(gy1) > gyroThreshold || abs(gy2) > gyroThreshold) {
    return "Walking/Running";
  }
  return "Unknown";
}

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Connected to Wi-Fi");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Initialize ThingsBoard MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  while (!mqttClient.connected()) {
    Serial.println("Connecting to ThingsBoard...");
    if (mqttClient.connect("ESP32", thingsboardToken, NULL)) {
      Serial.println("Connected to ThingsBoard");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }

  // Initialize MPU6050 sensors
  Wire.begin();
  
  mpu1.initialize();
  if (!mpu1.testConnection()) {
    Serial.println("MPU6050_1 connection failed");
  } else {
    Serial.println("MPU6050_1 connection successful");
  }
  
  mpu2.initialize();
  if (!mpu2.testConnection()) {
    Serial.println("MPU6050_2 connection failed");
  } else {
    Serial.println("MPU6050_2 connection successful");
  }
}

void loop() {
  if (!mqttClient.connected()) {
    mqttClient.connect("ESP32", thingsboardToken, NULL);
  }

  mqttClient.loop(); // Ensure MQTT client stays active

  // Read EMG data
  int emgValue = analogRead(emgPin);  // EMG sensor reading

  // Read FSR sensor data
  int fsrHeelValue = analogRead(fsrHeelPin);  // Heel FSR sensor reading
  int fsrToeValue = analogRead(fsrToePin);    // Toe FSR sensor reading

  // Determine gait phase based on FSR readings
  String gaitPhase = getGaitPhase(fsrHeelValue, fsrToeValue);
  Serial.println("Gait Phase: " + gaitPhase);

  // Read gyroscope and accelerometer data from both MPU6050 sensors
  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  int16_t ax2, ay2, az2, gx2, gy2, gz2;

  // Get data from first MPU6050 sensor
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  
  // Get data from second MPU6050 sensor
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

  // Convert raw accelerometer data to "g" values (gravity) for both sensors
  float accelX1 = ax1 / 16384.0;  // Conversion factor for accelerometer data
  float accelY1 = ay1 / 16384.0;
  float accelZ1 = az1 / 16384.0;
  
  float accelX2 = ax2 / 16384.0;
  float accelY2 = ay2 / 16384.0;
  float accelZ2 = az2 / 16384.0;

  // Get temperature from both MPU6050 sensors
  int16_t tempRaw1 = mpu1.getTemperature();
  float temperature1 = tempRaw1 / 340.0 + 36.53;  // Formula from MPU6050 datasheet
  
  int16_t tempRaw2 = mpu2.getTemperature();
  float temperature2 = tempRaw2 / 340.0 + 36.53;

  // Determine motion state based on gyroscope data
  String motionState = getMotionState(gy1, gy2);
  Serial.println("Motion State: " + motionState);

  // Print all sensor readings to serial monitor before sending to ThingSpeak
  Serial.println("Sending the following values to ThingSpeak:");
  Serial.println("EMG Value: " + String(emgValue));
  Serial.println("FSR Heel Value: " + String(fsrHeelValue));
  Serial.println("FSR Toe Value: " + String(fsrToeValue));

  Serial.println("MPU6050_1 Gyro X: " + String(gx1));
  Serial.println("MPU6050_1 Gyro Y: " + String(gy1));
  Serial.println("MPU6050_1 Gyro Z: " + String(gz1));
  Serial.println("MPU6050_1 Accel X: " + String(accelX1));
  Serial.println("MPU6050_1 Accel Y: " + String(accelY1));
  Serial.println("MPU6050_1 Accel Z: " + String(accelZ1));
  Serial.println("MPU6050_1 Temperature: " + String(temperature1));

  Serial.println("MPU6050_2 Gyro X: " + String(gx2));
  Serial.println("MPU6050_2 Gyro Y: " + String(gy2));
  Serial.println("MPU6050_2 Gyro Z: " + String(gz2));
  Serial.println("MPU6050_2 Accel X: " + String(accelX2));
  Serial.println("MPU6050_2 Accel Y: " + String(accelY2));
  Serial.println("MPU6050_2 Temperature: " + String(temperature2));

  // Send data to ThingSpeak
  ThingSpeak.setField(1, emgValue);       // EMG data
  ThingSpeak.setField(2, fsrHeelValue);   // Heel pressure data
  ThingSpeak.setField(3, fsrToeValue);    // Toe pressure data
  ThingSpeak.setField(4, gx1);            // Gyroscope X (MPU1)
  ThingSpeak.setField(5, gy1);            // Gyroscope Y (MPU1)
  ThingSpeak.setField(6, gz1);            // Gyroscope Z (MPU1)
  ThingSpeak.setField(7, accelX1);        // Accelerometer X (MPU1)
  ThingSpeak.setField(8, temperature1);   // Temperature (MPU1)
  
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  
  if (x == 200) {
    Serial.println("Data successfully sent to ThingSpeak");
  } else {
    Serial.println("Error sending data to ThingSpeak: HTTP code " + String(x));
  }

  // Prepare MQTT payload for ThingsBoard
  String payload = "{";
  payload += "\"emg\":" + String(emgValue) + ",";
  payload += "\"fsrHeel\":" + String(fsrHeelValue) + ",";
  payload += "\"fsrToe\":" + String(fsrToeValue) + ",";
  payload += "\"gyroX1\":" + String(gx1) + ",";
  payload += "\"gyroY1\":" + String(gy1) + ",";
  payload += "\"gyroZ1\":" + String(gz1) + ",";
  payload += "\"accelX1\":" + String(accelX1) + ",";
  payload += "\"temperature1\":" + String(temperature1) + ",";
  payload += "\"gaitPhase\":\"" + gaitPhase + "\",";
  payload += "\"motionState\":\"" + motionState + "\"";
  payload += "}";

  // Publish the payload to ThingsBoard
 
    ThingSpeak.setStatus(gaitPhase + ", " + motionState);
  mqttClient.publish("v1/devices/me/telemetry", payload.c_str());
  Serial.println("Data successfully sent to ThingsBoard");

  // Delay before the next loop iteration
  delay(2000);  // 20 seconds (ThingSpeak requires 15 seconds minimum)
}
