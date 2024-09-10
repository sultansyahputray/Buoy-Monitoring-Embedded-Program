#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <GY521.h>
#include <TinyGPSPlus.h>

GY521 sensor(0x68);
TinyGPSPlus gps;

#define END_NODE_NUMBER 2

// Define pin on ESP32
const int csPin = 5;      // LoRa radio chip select
const int resetPin = 4;   // LoRa radio reset
const int irqPin = 25;    // Interrupt pin
const int voltPin = 34;   // Voltage sensor
const int tempPin = 26;   // Temp and humidity sensor
const int LDRPin = 35;    // LDR sensor

// Define buoy coordinates
// Buoy 14
// double longitude = -7.0048399;
// double latitude = 112.6828811;
// uint32_t satelite; 

// Buoy 15
double longitude = -7.0158469;
double latitude = 112.6737149;
uint32_t satelite;

float accPitch;
float accRoll;
float vibrate;
float tegangan;
float suhu;
float humidity;
uint16_t cahaya;

// Variables to store the initial pitch and roll values
float initialPitch = 0;
float initialRoll = 0;
bool isFirstRead = true;

unsigned long sensslowMillis = 0;
unsigned long senssmidMillis = 0;
unsigned long senssfstMillis = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);
  Serial.println("LoRa Sender Test");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  sensor_init();
}

void loop() {
  sensor_read();

  // Payload
  String payload = String(END_NODE_NUMBER) + ";" + String(accPitch) + ";" + String(accRoll) + ";" + String(tegangan) + ";" + String(suhu) + ";" + String(humidity) + ";" + String(cahaya) + ";" + String(longitude, 7) + ";" + String(latitude, 7) + ";" + String(satelite);

  Serial.print("Sending packet: ");
  Serial.println(payload);

  // Send the payload
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  delay(1000);
}

void sensor_init() {
  imu_init();
}

void sensor_read() {
  unsigned long currentMillis = millis();

  if (currentMillis - senssfstMillis >= 10) {
    senssfstMillis = currentMillis;
    volt_read();
    temp_read();
  }

  if (currentMillis - senssmidMillis >= 100) {
    senssmidMillis = currentMillis;  
  }

  if (currentMillis - sensslowMillis >= 1000) {
    sensslowMillis = currentMillis;

    imu_read();
    lumen_read();
  }
}

void imu_init() {
  Wire.begin();
  delay(100);

  while (sensor.wakeup() == false) {
    Serial.print(millis());
    Serial.println("\tCouldn connect to sensor IMU");
    delay(1000);
  }

  sensor.setAccelSensitivity(3);
  sensor.setGyroSensitivity(0);
  sensor.setThrottle(false);
}

void imu_read() {
  sensor.read();
  float ax = (sensor.getAccelX() * 9.80665);
  float ay = (sensor.getAccelY() * 9.80665);
  float az = (sensor.getAccelZ() * 9.80665);

  vibrate = abs(ax) + abs(ay) + abs(az);
  float pitch = (180 * atan2(ax, sqrt(ay*ay + az*az))/PI) * (-1);
  float roll = 180 * atan2(ay, sqrt(ax*ax + az*az))/PI;

  // Check if this is the first read to capture initial pitch and roll values
  if (isFirstRead) {
    initialPitch = pitch;
    initialRoll = roll;
    isFirstRead = false;
  }

  // Calculate relative pitch and roll to the initial position
  accPitch = pitch - initialPitch;
  accRoll = roll - initialRoll;

  // Optional: Print values to the serial monitor for debugging
  Serial.print("Relative Pitch: ");
  Serial.println(accPitch);
  Serial.print("Relative Roll: ");
  Serial.println(accRoll);
}

void volt_read() {
  static float last_tegangan;
  float alpha = 0.97;
  int tegangan_raw;
  tegangan_raw = (alpha * last_tegangan) + ((1.0 - alpha) * analogRead(voltPin));
  last_tegangan = tegangan_raw;
  tegangan = map(tegangan_raw, 592, 900, 900, 1250);
  tegangan = tegangan / 100.0;
  tegangan = tegangan < 0.0 ? 0.0 : tegangan;
}

void temp_read() {
  static int last_Vo;
  float alpha = 0.85;
  int Vo;
  float R1 = 10000;
  float logR2, R2, T, Dp = 23.5;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

  Vo = (alpha * last_Vo) + ((1.0 - alpha) * analogRead(tempPin));
  last_Vo = Vo;
  R2 = R1 * (4096.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  suhu = T - 273.15 + 3.0;
  humidity = 0.4 * 100 * (exp((17.625 * Dp) / (243.04 + Dp)) / exp((17.625 * suhu) / (243.04 + suhu)));
}

void lumen_read(){
  cahaya = analogRead(LDRPin);
}

void gps_read(){
  static unsigned long gps_milis;
  unsigned long current_millis = millis();

  while (Serial2.available() > 0)
    gps.encode(Serial2.read());

  if(current_millis - gps_milis >= 1000){
    gps_milis = current_millis;
    if(gps.location.isValid()){
      longitude = gps.location.lng();
      latitude = gps.location.lat();
      Serial.print("Longitude: ");
      Serial.println(longitude, 6);
      Serial.print("Latitude: ");
      Serial.println(latitude, 6);
    } else {
      Serial.println("Waiting for GPS fix...");
    }

    if(gps.satellites.isValid()){
      satelite = gps.satellites.value();
      Serial.print("Satellites: ");
      Serial.println(satelite);
    }
  }
}
