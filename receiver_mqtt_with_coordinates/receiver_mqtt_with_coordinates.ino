#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
// #include "BluetoothSerial.h"

#define GATEWAY_NUMBER 1
#define NUM_ITERATIONS 1
#define TIMEOUT 60000
#define FREQ 433E6
#define SPREADING_FACTOR 12 // 12
#define BANDWIDTH 41700 // 41,7 kHz
#define CODING_RATE 8  // Coding Rate 8
#define PREAMBLE_LENGTH 20 // 12

TinyGPSPlus gps;
// BluetoothSerial SerialBT;

// Define WiFi credentials
const char* ssid = "Ntah";
const char* password = "whuu5663";

IPAddress staticIP(192, 168, 1, 6);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);

// Define MQTT broker settings
const char* mqtt_server = "192.168.130.221"; // Broker IP
const int mqtt_port = 1883;

// Define MQTT topics
const char* buoy1_topic = "buoy/1";

// Define the pins of LoRa module
const int csPin = 5;     // LoRa radio chip select
const int resetPin = 4;  // LoRa radio reset
const int irqPin = 25;   // Must be a hardware interrupt pin

unsigned long senssmidMillis = 0;
uint32_t satelite;
int RSSI;
float SNR;
float voltage;
double longitude;
double latitude;
int buoyNumber;
int counter = 0;
bool buoyStatus = false;
bool buoy1Active = false;
unsigned long lastUpdateTime = 0;
bool zeroDataSent = false;
String tanggal, waktu;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(250000);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  // SerialBT.begin("Gateway_1");
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);
  Serial.println("LoRa Receiver Test");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(SPREADING_FACTOR); 
  LoRa.setSignalBandwidth(BANDWIDTH);  
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setPreambleLength(PREAMBLE_LENGTH);

  sensor_init();

  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  connectMQTT();
}

void loop() {
  sensor_read();

  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  updateBuoyData();

  if (buoyStatus) {
    sendDataToMQTT();
    zeroDataSent = false;
  }
}

void sensor_init() {
  gps_init(); 
}

void sensor_read() {
  unsigned long currentMillis = millis();

  if (currentMillis - senssmidMillis >= 100) {
    senssmidMillis = currentMillis;  
    gps_read();
  }
}

void gps_init(){
    Serial.println(gps.libraryVersion());
    // SerialBT.println(gps.libraryVersion()); 
    if(gps.libraryVersion() != 0){
        // SerialBT.println("gps init SUCCESS"); 
    } else {
        // SerialBT.println("gps init FAILED");  
    }
    delay(100);
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
      tanggal = String(gps.date.day()) + "/"+ String(gps.date.month()) + "/" + String(gps.date.year());
      waktu = String(gps.time.hour() + 7) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      Serial.print("Longitude: ");
      Serial.println(longitude, 6);
      // SerialBT.print("Longitude: ");  
      // SerialBT.println(longitude, 6);  

      Serial.print("Latitude: ");
      Serial.println(latitude, 6);
      // SerialBT.print("Latitude: ");  
      // SerialBT.println(latitude, 6);  

      // SerialBT.print("Tanggal: ");  
      // SerialBT.println(tanggal);  
      // SerialBT.print("Waktu: "); 
      // SerialBT.println(waktu);
    } else {
      Serial.println("Waiting for GPS fix...");  
    }

    if(gps.satellites.isValid()){
      satelite = gps.satellites.value();
      // Serial.print("Satellites: ");
      // Serial.println(satelite);
      // SerialBT.print("Satellites: ");  
      // SerialBT.println(satelite);  
    }  
  }
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

      parseData(receivedData);
      printBuoyData();

      buoy1Active = true;

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

  // if want to config ip wifi
  // if (WiFi.config(staticIP, gateway, subnet, dns, dns) == false) {
  //   Serial.println("config failed");
  // }
  //

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

void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("LoRaClient")) { 
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
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
  voltage = data.substring(separator3 + 1, separator4).toFloat();
  float temperature = data.substring(separator4 + 1, separator5).toFloat();
  float hum = data.substring(separator5 + 1, separator6).toFloat();
  int light = data.substring(separator6 + 1, separator7).toInt();
}

void sendDataToMQTT() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Sending data to MQTT");

    String buoy1Data = buoy1Active ? String(GATEWAY_NUMBER) + "," + String(longitude, 7) + "," + String(latitude, 7) + "," + 
                       String(tanggal) + "," + String(waktu) + "," + String(RSSI) + "," + String(SNR) + "," + String(voltage)
                       : String(GATEWAY_NUMBER) + ",0,0" + "," + String(tanggal) + "," + String(waktu) + "," + "0,0,0";
    client.publish(buoy1_topic, buoy1Data.c_str());

    counter = 0;
    buoyStatus = false;
    buoy1Active = false;
  } else {
    Serial.println("WiFi Disconnected! Trying to reconnect...");
    connectWiFi();
  }
}

void printBuoyData() {
  Serial.print(", Longitude: ");
  Serial.println(longitude, 6);
  Serial.print(", Latitude: ");
  Serial.println(latitude, 6);
  Serial.print(", Tanggal: ");
  Serial.println(tanggal);
  Serial.print(", Waktu: ");
  Serial.println(waktu);
  Serial.print(", RSSI: ");
  Serial.println(RSSI);
  Serial.print(", SNR: ");
  Serial.println(SNR);
  Serial.print(", Voltage: ");
  Serial.println(voltage);
}
