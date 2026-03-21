/*
Features:
-	Reads acceleration and gyroscope data from an MPU6050 sensor.
-	Uses a Kalman filter to smooth and correct sensor readings.
-	Reads ambient light levels using an LDR sensor.
-	Communicates sensor data wirelessly via Bluetooth Classic.
-	Sends processed values over serial for debugging.
*/


#include <Adafruit_MPU6050.h> #include <Adafruit_Sensor.h>
 
#include <Wire.h>
#include "BluetoothSerial.h"


Adafruit_MPU6050 mpu;	// MPU6050 sensor object BluetoothSerial SerialBT; // Bluetooth serial object

const int LDRSensor = 15; // LDR sensor pin


// Kalman filter variables (shared matrix for both axes – simple but works) float x_angle = 0, y_angle = 0;
float x_bias = 0, y_bias = 0;
float P[2][2] = { { 1, 0 }, { 0, 1 } };


// Kalman tuning (quick, less smooth) float q_angle = 0.01;
float q_bias	= 0.01;
float r_measure = 0.01;


float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias) { float rate = newRate - bias;
angle += dt * rate;


// Update covariance
P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + q_angle); P[0][1] -= dt * P[1][1];
 
P[1][0] -= dt * P[1][1];
P[1][1] += q_bias * dt;


// Kalman gain
float S = P[0][0] + r_measure;
float K[2] = { P[0][0] / S, P[1][0] / S };


// Update with measurement float y = newAngle - angle;
angle += K[0] * y; bias += K[1] * y;

// Update covariance
P[0][0] -= K[0] * P[0][0];
P[0][1] -= K[0] * P[0][1];
P[1][0] -= K[1] * P[0][0];
P[1][1] -= K[1] * P[0][1];


return angle;
}


void setup() {
Serial.begin(115200); delay(500);
 
Serial.println("Starting Bluetooth...");
if (!SerialBT.begin("ESP32_BT")) {	// Start BT Classic SPP Serial.println("Bluetooth init FAILED!");
while (true) {
delay(1000); // Hard stop if BT fails
}
}
Serial.println("Bluetooth started, device name: ESP32_BT");


// Init MPU6050
if (!mpu.begin()) {
Serial.println("MPU6050 not found!"); while (true) {
delay(10);
}
}


mpu.setAccelerometerRange(MPU6050_RANGE_8_G); mpu.setGyroRange(MPU6050_RANGE_500_DEG);
mpu.setFilterBandwidth(MPU6050_BAND_10_HZ); delay(100);

pinMode(LDRSensor, INPUT);
}
 
void loop() {
static unsigned long prevTime = millis(); static unsigned long lastSendMs = 0;
static bool lastClientState	= false;


unsigned long now = millis();
float dt = (now - prevTime) / 1000.0f; prevTime = now;

// Check BT client connection state bool hasClient = SerialBT.hasClient(); if (hasClient != lastClientState) {
lastClientState = hasClient; Serial.print("BT client ");
Serial.println(hasClient ? "CONNECTED" : "DISCONNECTED");
}


// If no BT client is connected, don't spam BT stack – just chill if (!hasClient) {
delay(20); return;
}


// Throttle sending rate to ~50 Hz (every 20 ms) if (now - lastSendMs < 20) {
 
return;
}
lastSendMs = now;


// Read sensors
sensors_event_t a, g, temp; mpu.getEvent(&a, &g, &temp);

int ldrValue = analogRead(LDRSensor);
int outputValue = (ldrValue < 2000) ? 0 : 1;


float filteredX = kalmanFilter(a.acceleration.x, g.gyro.x, dt, x_angle, x_bias); float filteredY = kalmanFilter(a.acceleration.y, g.gyro.y, dt, y_angle, y_bias);

String btData = String(filteredX, 2) + "," + String(filteredY, 2) + "," +
String(outputValue);


// Send over BT & USB for debugging SerialBT.println(btData);
Serial.println(btData);
}
