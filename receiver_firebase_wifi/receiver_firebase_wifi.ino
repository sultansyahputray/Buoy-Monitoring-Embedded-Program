#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define GATEWAY_NUMBER 1
#define NUM_ITERATIONS 15
#define TIMEOUT 300000
#define FREQ 433E6
#define SPREADING_FACTOR 12
#define BANDWIDTH 41700 // 15,6 kHz
#define CODING_RATE 8  // Coding Rate 4/8
#define PREAMBLE_LENGTH 12

// Define WiFi credentials
const char* ssid = "CBN";
const char* password = "Tanaya794";

// Define Firebase details
const String FIREBASE_HOST = "https://gateway-data-gsm-default-rtdb.asia-southeast1.firebasedatabase.app/";
const String FIREBASE_SECRET = "7ydfdVT7E35GpD5J4Px4bHy9hPaesiPvfK8TSUTW";

// Define LoRa pins
const int csPin = 5;     // LoRa radio chip select
const int resetPin = 4;  // LoRa radio reset
const int irqPin = 25;   // Must be a hardware interrupt pin

int RSSI;
float SNR;
int buoyNumber;
int counter = 0;
bool buoyStatus = false;
bool buoy1Active = false;
bool buoy2Active = false;
unsigned long lastUpdateTime = 0;
bool zeroDataSent = false;

// Variables for buoy 1
float accPitch1 = 0;
float accRoll1 = 0;
float tegangan1 = 0;
float suhu1 = 0;
float humidity1 = 0;
int light1 = 0;
double longitude1 = 0;
double latitude1 = 0;
String tanggal1;
String waktu1;
int RSSI1 = 0;
float SNR1 = 0;

// Variables for buoy 2
float accPitch2 = 0;
float accRoll2 = 0;
float tegangan2 = 0;
float suhu2 = 0;
float humidity2 = 0;
int light2 = 0;
double longitude2 = 0;
double latitude2 = 0;
String tanggal2;
String waktu2;
int RSSI2 = 0;
float SNR2 = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);
  Serial.println("LoRa Receiver Test");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Set LoRa parameters
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setPreambleLength(PREAMBLE_LENGTH);

  connectWiFi();

  // Configure NTP with GMT+7 (WIB)
  configTime(25200, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("Fetching time from NTP server...");

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.printf("Current time: %04d-%02d-%02d %02d:%02d:%02d\n",
                timeinfo.tm_year + 1900,
                timeinfo.tm_mon + 1,
                timeinfo.tm_mday,
                timeinfo.tm_hour,
                timeinfo.tm_min,
                timeinfo.tm_sec);
}

void loop() {
  updateBuoyData();

  if (buoyStatus) {
    sendDataToFirebase();
    zeroDataSent = false;
  } else if (millis() - lastUpdateTime >= TIMEOUT) {
    sendZeroDataToFirebase();
    Serial.println("Zero data");
    lastUpdateTime = millis();
  }
}

String getFormattedDateTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "Invalid time";
  }

  char buffer[30];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          timeinfo.tm_year + 1900,
          timeinfo.tm_mon + 1,
          timeinfo.tm_mday,
          timeinfo.tm_hour,
          timeinfo.tm_min,
          timeinfo.tm_sec);
  return String(buffer);
}

void updateBuoyData() {
  bool dataReceived = false;

  for (int i = 0; i < NUM_ITERATIONS; i++) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      dataReceived = true;
      String receivedData = "";
      while (LoRa.available()) {
        receivedData += (char)LoRa.read();
      }
      RSSI = LoRa.packetRssi();
      SNR = LoRa.packetSnr();
      Serial.print("Received packet: ");
      Serial.println(receivedData);

      parseData(receivedData);
      if (buoyNumber == 1) buoy1Active = true;
      if (buoyNumber == 2) buoy2Active = true;

      counter++;
    }
  }

  if (counter >= NUM_ITERATIONS) {
    buoyStatus = true;
  }

  if (dataReceived) {
    lastUpdateTime = millis();
    zeroDataSent = false;
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("Connected to: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void parseData(String data) {
  int separator1 = data.indexOf(';');
  int separator2 = data.indexOf(';', separator1 + 1);
  int separator3 = data.indexOf(';', separator2 + 1);
  int separator4 = data.indexOf(';', separator3 + 1);
  int separator5 = data.indexOf(';', separator4 + 1);
  int separator6 = data.indexOf(';', separator5 + 1);
  int separator7 = data.indexOf(';', separator6 + 1);
  int separator8 = data.indexOf(';', separator7 + 1);
  int separator9 = data.indexOf(';', separator8 + 1);
  int separator10 = data.indexOf(';', separator9 + 1);

  buoyNumber = data.substring(0, separator1).toInt();
  float pitch = data.substring(separator1 + 1, separator2).toFloat();
  float roll = data.substring(separator2 + 1, separator3).toFloat();
  float voltage = data.substring(separator3 + 1, separator4).toFloat();
  float temperature = data.substring(separator4 + 1, separator5).toFloat();
  float hum = data.substring(separator5 + 1, separator6).toFloat();
  int light = data.substring(separator6 + 1, separator7).toInt();
  double longitude = data.substring(separator7 + 1, separator8).toDouble();
  double latitude = data.substring(separator8 + 1, separator9).toDouble();
  String tanggal = data.substring(separator9 + 1, separator10);
  String waktu = data.substring(separator10 + 1);

  if (buoyNumber == 1) {
    accPitch1 = pitch;
    accRoll1 = roll;
    tegangan1 = voltage;
    suhu1 = temperature;
    humidity1 = hum;
    light1 = light;
    longitude1 = longitude;
    latitude1 = latitude;
    tanggal1 = tanggal;
    waktu1 = waktu;
    RSSI1 = RSSI;
    SNR1 = SNR;
  } else if (buoyNumber == 2) {
    accPitch2 = pitch;
    accRoll2 = roll;
    tegangan2 = voltage;
    suhu2 = temperature;
    humidity2 = hum;
    light2 = light;
    longitude2 = longitude;
    latitude2 = latitude;
    tanggal2 = tanggal;
    waktu2 = waktu;
    RSSI2 = RSSI;
    SNR2 = SNR;
  }
}

void sendDataToFirebase() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    if (buoy1Active) {
      String url1 = FIREBASE_HOST + "data_new/buoy1.json?auth=" + FIREBASE_SECRET;

      StaticJsonDocument<256> json1;
      json1["buoy_number"] = 1;
      json1["pitch"] = accPitch1;
      json1["roll"] = accRoll1;
      json1["voltage"] = tegangan1;
      json1["temperature"] = suhu1;
      json1["humidity"] = humidity1;
      json1["light"] = light1;
      json1["longitude"] = longitude1;
      json1["latitude"] = latitude1;
      json1["tanggal_node"] = tanggal1;
      json1["waktu_node"] = waktu1;
      json1["RSSI"] = RSSI1;
      json1["SNR"] = SNR1;
      json1["timestamp"] = getFormattedDateTime();

      String payload1;
      serializeJson(json1, payload1);

      http.begin(url1);
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode1 = http.POST(payload1);

      if (httpResponseCode1 > 0) {
        Serial.printf("Data for Buoy 1 sent successfully: %d\n", httpResponseCode1);
      } else {
        Serial.printf("Error sending data for Buoy 1: %s\n", http.errorToString(httpResponseCode1).c_str());
      }
      http.end();
    }

    if (buoy2Active) {
      String url2 = FIREBASE_HOST + "data_new/buoy2.json?auth=" + FIREBASE_SECRET;

      StaticJsonDocument<256> json2;
      json2["buoy_number"] = 2;
      json2["pitch"] = accPitch2;
      json2["roll"] = accRoll2;
      json2["voltage"] = tegangan2;
      json2["temperature"] = suhu2;
      json2["humidity"] = humidity2;
      json2["light"] = light2;
      json2["longitude"] = longitude2;
      json2["latitude"] = latitude2;
      json2["tanggal_node"] = tanggal2;
      json2["waktu_node"] = waktu2;
      json2["RSSI"] = RSSI2;
      json2["SNR"] = SNR2;
      json2["timestamp"] = getFormattedDateTime();

      String payload2;
      serializeJson(json2, payload2);

      http.begin(url2);
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode2 = http.POST(payload2);

      if (httpResponseCode2 > 0) {
        Serial.printf("Data for Buoy 2 sent successfully: %d\n", httpResponseCode2);
      } else {
        Serial.printf("Error sending data for Buoy 2: %s\n", http.errorToString(httpResponseCode2).c_str());
      }
      http.end();
    }

    // Reset flags
    counter = 0;
    buoyStatus = false;
    buoy1Active = false;
    buoy2Active = false;
  } else {
    Serial.println("WiFi Disconnected! Trying to reconnect...");
    connectWiFi();
  }
}

void sendZeroDataToFirebase() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url1 = FIREBASE_HOST + "data_new/zero.json?auth=" + FIREBASE_SECRET;

    StaticJsonDocument<256> json1;
    json1["buoy_number"] = 0;
    json1["pitch"] = 0;
    json1["roll"] = 0;
    json1["voltage"] = 0;
    json1["temperature"] = 0;
    json1["humidity"] = 0;
    json1["light"] = 0;
    json1["longitude"] = 0;
    json1["latitude"] = 0;
    json1["tanggal_node"] = 0;
    json1["waktu_node"] = 0;
    json1["RSSI"] = 0;
    json1["SNR"] = 0;
    json1["timestamp"] = getFormattedDateTime();
    
    String payload1;
    serializeJson(json1, payload1);
    http.begin(url1);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode1 = http.POST(payload1);

    if (httpResponseCode1 > 0) {
      Serial.printf("Zero data sent successfully: %d\n", httpResponseCode1);
    } else {
      Serial.printf("Error sending zero data: %s\n", http.errorToString(httpResponseCode1).c_str());
    }
    http.end();

    zeroDataSent = true;
  } else {
    Serial.println("WiFi Disconnected! Trying to reconnect...");
    connectWiFi();
  }
}


void printBuoyData() {
  Serial.print("Buoy 1 - Pitch: ");
  Serial.print(accPitch1);
  Serial.print(", Roll: ");
  Serial.print(accRoll1);
  Serial.print(", Voltage: ");
  Serial.print(tegangan1);
  Serial.print(", Temperature: ");
  Serial.print(suhu1);
  Serial.print(", Humidity: ");
  Serial.println(humidity1);
  Serial.print(", LDR: ");
  Serial.println(light1);
  Serial.print(", Longitude: ");
  Serial.println(longitude1);
  Serial.print(", Latitude: ");
  Serial.println(latitude1);
  Serial.print(", Tanggal: ");
  Serial.println(tanggal1);
  Serial.print(", Waktu: ");
  Serial.println(waktu1);
  Serial.print(", RSSI: ");
  Serial.println(RSSI1);
  Serial.print(", SNR: ");
  Serial.println(SNR1);

  Serial.print("Buoy 2 - Pitch: ");
  Serial.print(accPitch2);
  Serial.print(", Roll: ");
  Serial.print(accRoll2);
  Serial.print(", Voltage: ");
  Serial.print(tegangan2);
  Serial.print(", Temperature: ");
  Serial.print(suhu2);
  Serial.print(", Humidity: ");
  Serial.println(humidity2);
  Serial.print(", LDR: ");
  Serial.println(light2);
  Serial.print(", Longitude: ");
  Serial.println(longitude2);
  Serial.print(", Latitude: ");
  Serial.println(latitude2);
  Serial.print(", Tanggal: ");
  Serial.println(tanggal2);
  Serial.print(", Waktu: ");
  Serial.println(waktu2);
  Serial.print(", RSSI: ");
  Serial.println(RSSI2);
  Serial.print(", SNR: ");
  Serial.println(SNR2);
}
