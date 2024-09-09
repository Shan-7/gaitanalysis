#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_LSM9DS1.h>

// Wi-Fi credentials
const char* ssid = "Galaxy S";
const char* password = "Frequency";

// ThingsBoard credentials
const char* thingsboardServer = "demo.thingsboard.io";
const char* accessToken = "gchq6ion4hrrc27q1cow"; // ThingsBoard token

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Define MAX30105 Sensor
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100]; // Infrared LED sensor data
uint32_t redBuffer[100];  // Red LED sensor data
int32_t bufferLength; // Data length
int32_t spo2; // SpO2 value
int8_t validSPO2; // Indicator to show if the SpO2 calculation is valid
int32_t heartRate; // Heart rate value
int8_t validHeartRate; // Indicator to show if the heart rate calculation is valid

// Define KY-028 pins
int analogPin = A0;
float temp; // Variable for temperature data

// Define LSM9DS1 (Gyroscope) Sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
sensors_event_t gyro, accel, mag, tempEvent; // Variables for LSM9DS1 data

// Function to connect to Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

// Function to reconnect to ThingsBoard
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    if (client.connect("NodeMCU", accessToken, NULL)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(thingsboardServer, 1883);

  // Initialize the MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 sensor not found. Please check wiring/power.");
    while (1);
  }

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  byte sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure MAX30105 sensor

  // Initialize LSM9DS1 (Gyroscope) Sensor
  if (!lsm.begin()) {
    Serial.println("Failed to initialize LSM9DS1 sensor! Check your wiring.");
    while (1);
  }

  // Set up the KY-028 sensor (no special initialization needed)
}

// Loop function
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  bufferLength = 100; // Buffer length for MAX30105

  // Read samples from MAX30105 sensor
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) {
      particleSensor.check(); // Check the sensor for new data
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // Move to next sample
  }

  // Calculate heart rate and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Read temperature data from KY-028
  int v_input = analogRead(analogPin);
  temp = -1 * (125.315 - 0.175529 * v_input - 2.5); // Adjusted formula for temperature

  // Read data from LSM9DS1 (Gyroscope) sensor
  lsm.getEvent(&accel, &mag, &gyro, &tempEvent); // Retrieve sensor data

  // Print values to serial monitor
  Serial.print("Heart Rate: ");
  Serial.print(heartRate);
  Serial.print(", SpO2: ");
  Serial.print(spo2);
  Serial.print(", Temp: ");
  Serial.print(temp);
  Serial.print(", Gyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(", Gyro Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print(", Gyro Z: ");
  Serial.println(gyro.gyro.z);

  // Prepare JSON payload for ThingsBoard
  String payload = "{";
  payload += "\"heartRate\":";
  payload += heartRate;
  payload += ",\"spo2\":";
  payload += spo2;
  payload += ",\"temperature\":";
  payload += temp;
  payload += ",\"gyro_x\":";
  payload += gyro.gyro.x;
  payload += ",\"gyro_y\":";
  payload += gyro.gyro.y;
  payload += ",\"gyro_z\":";
  payload += gyro.gyro.z;
  payload += "}";

  // Publish data to ThingsBoard
  if (client.publish("v1/devices/me/telemetry", payload.c_str())) {
    Serial.println("Data sent to ThingsBoard");
  } else {
    Serial.println("Failed to send data");
  }

  delay(1000); // 1 second delay
}
