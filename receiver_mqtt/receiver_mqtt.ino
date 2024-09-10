#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define GATEWAY_NUMBER 1
#define NUM_ITERATIONS 10 
#define TIMEOUT 60000

// Define WiFi credentials
const char* ssid = "EROS";
const char* password = "EROS1NASIONAL";

IPAddress staticIP(192, 168, 1, 6);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);

// Define MQTT broker settings
const char* mqtt_server = "192.168.1.10"; // Broker IP
const int mqtt_port = 1883;

// Define MQTT topics
const char* buoy1_topic = "buoy/1";
const char* buoy2_topic = "buoy/2";

// Define the pins of LoRa module
const int csPin = 5;     // LoRa radio chip select
const int resetPin = 4;  // LoRa radio reset
const int irqPin = 25;   // Must be a hardware interrupt pin

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

// Variables for buoy 2
float accPitch2 = 0;
float accRoll2 = 0;
float tegangan2 = 0;
float suhu2 = 0;
float humidity2 = 0;
int light2 = 0;
double longitude2 = 0;
double latitude2 = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);
  Serial.println("LoRa Receiver Test");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  connectMQTT();
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  Serial.println("Update data buoy");
  Serial.print("Buoy Status: ");
  Serial.println(buoyStatus);
  updateBuoyData();

  if (buoyStatus) {
    sendDataToMQTT();
    zeroDataSent = false;
  } else if (millis() - lastUpdateTime >= TIMEOUT && !zeroDataSent) {
    sendZeroDataToMQTT();
    zeroDataSent = true;
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
      Serial.print("Received packet: ");
      Serial.println(receivedData);

      parseData(receivedData);
      printBuoyData();

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

  buoyNumber = data.substring(0, separator1).toInt();
  float pitch = data.substring(separator1 + 1, separator2).toFloat();
  float roll = data.substring(separator2 + 1, separator3).toFloat();
  float voltage = data.substring(separator3 + 1, separator4).toFloat();
  float temperature = data.substring(separator4 + 1, separator5).toFloat();
  float hum = data.substring(separator5 + 1, separator6).toFloat();
  int light = data.substring(separator6 + 1, separator7).toInt();
  double longitude = data.substring(separator7 + 1, separator8).toDouble();
  double latitude = data.substring(separator8 + 1).toDouble();

  if (buoyNumber == 1) {
    accPitch1 = pitch;
    accRoll1 = roll;
    tegangan1 = voltage;
    suhu1 = temperature;
    humidity1 = hum;
    light1 = light;
    longitude1 = longitude;
    latitude1 = latitude;
  } else if (buoyNumber == 2) {
    accPitch2 = pitch;
    accRoll2 = roll;
    tegangan2 = voltage;
    suhu2 = temperature;
    humidity2 = hum;
    light2 = light;
    longitude2 = longitude;
    latitude2 = latitude;
  }
}

void sendDataToMQTT() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Sending data to MQTT");

    String buoy1Data = buoy1Active ? String(GATEWAY_NUMBER) + "," + String(accPitch1) + "," + String(accRoll1) + "," +
                       String(tegangan1) + "," + String(suhu1) + "," + String(humidity1) + "," + String(light1) + "," + String(longitude1, 7) + "," + String(latitude1, 7)
                       : String(GATEWAY_NUMBER) + ",0,0,0,0,0,0,0,0";
    client.publish(buoy1_topic, buoy1Data.c_str());

    String buoy2Data = buoy2Active ? String(GATEWAY_NUMBER) + "," + String(accPitch2) + "," + String(accRoll2) + "," +
                       String(tegangan2) + "," + String(suhu2) + "," + String(humidity2) + "," + String(light2) + "," + String(longitude2, 7) + "," + String(latitude2, 7)
                       : String(GATEWAY_NUMBER) + ",0,0,0,0,0,0,0,0";
    client.publish(buoy2_topic, buoy2Data.c_str());

    counter = 0;
    buoyStatus = false;
    buoy1Active = false;
    buoy2Active = false;
  } else {
    Serial.println("WiFi Disconnected! Trying to reconnect...");
    connectWiFi();
  }
}

void sendZeroDataToMQTT() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Sending zero data to MQTT");

    String zeroBuoyData = String(GATEWAY_NUMBER) + ",0,0,0,0,0,0,0,0";

    // Publish zero data for both buoy1 and buoy2
    client.publish(buoy1_topic, zeroBuoyData.c_str());
    client.publish(buoy2_topic, zeroBuoyData.c_str());
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
}
