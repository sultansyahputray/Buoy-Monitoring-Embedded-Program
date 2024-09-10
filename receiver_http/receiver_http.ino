#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

#define GATEWAY_NUMBER 1
#define NUM_ITERATIONS 10 

// Define WiFi credentials
const char* ssid = "C-109";
const char* password = "tanaya123";
const String serverURL = "http://192.168.1.5/buoy/coba.php";

// Define the pins of LoRa module
const int csPin = 5;     // LoRa radio chip select
const int resetPin = 4;  // LoRa radio reset
const int irqPin = 25;   // Must be a hardware interrupt pin

int buoyNumber;
int counter = 0;
bool buoyStatus;

// Variables for buoy 1
float accPitch1 = 0;
float accRoll1 = 0;
float tegangan1 = 0;
float suhu1 = 0;
float humidity1 = 0;
int light1 = 0;
unsigned long buoy1LastUpdate = 0;

// Variables for buoy 2
float accPitch2 = 0;
float accRoll2 = 0;
float tegangan2 = 0;
float suhu2 = 0;
float humidity2 = 0;
int light2 = 0;
unsigned long buoy2LastUpdate = 0;

unsigned long lastSendMillis = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);
  Serial.println("LoRa Receiver Test");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // connectWiFi();
}

void loop() {
  Serial.println("update data buoy");
  Serial.print("Buoy Status: ");
  Serial.println(buoyStatus);
  updateBuoyData();

  if (buoyStatus) {
    // sendDataToServer();
  }
}

void updateBuoyData() {
  // Receive LoRa data
  for (int i = 0; i < NUM_ITERATIONS; i++) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String receivedData = "";
      while (LoRa.available()) {
        receivedData += (char)LoRa.read();
      }
      Serial.print("Received packet: ");
      Serial.println(receivedData);

      // Parse the received data
      parseData(receivedData);
      printBuoyData();

      counter++;
    }
  }

  if (counter >= NUM_ITERATIONS) {
    buoyStatus = true;
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_OFF);
  delay(1000);

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

  buoyNumber = data.substring(0, separator1).toInt();
  float pitch = data.substring(separator1 + 1, separator2).toFloat();
  float roll = data.substring(separator2 + 1, separator3).toFloat();
  float voltage = data.substring(separator3 + 1, separator4).toFloat();
  float temperature = data.substring(separator4 + 1, separator5).toFloat();
  float hum = data.substring(separator5 + 1, separator6).toFloat();
  int light = data.substring(separator6 + 1).toInt();

  if (buoyNumber == 1) {
    accPitch1 = pitch;
    accRoll1 = roll;
    tegangan1 = voltage;
    suhu1 = temperature;
    humidity1 = hum;
    light1 = light;
    buoy1LastUpdate = millis();
  } else if (buoyNumber == 2) {
    accPitch2 = pitch;
    accRoll2 = roll;
    tegangan2 = voltage;
    suhu2 = temperature;
    humidity2 = hum;
    light2 = light;
    buoy2LastUpdate = millis();
  }
}

void resetBuoy1() {
  accPitch1 = 0;
  accRoll1 = 0;
  tegangan1 = 0;
  suhu1 = 0;
  humidity1 = 0;
  light1 = 0;
}

void resetBuoy2() {
  accPitch2 = 0;
  accRoll2 = 0;
  tegangan2 = 0;
  suhu2 = 0;
  humidity2 = 0;
  light2 = 0;
}

void sendDataToServer() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Sending data to server");

    String postData = "gateway_number=" + String(GATEWAY_NUMBER) +
                      "&buoy_number=1&pitch=" + String(accPitch1) +
                      "&roll=" + String(accRoll1) +
                      "&tegangan=" + String(tegangan1) +
                      "&suhu=" + String(suhu1) +
                      "&humidity=" + String(humidity1);
    sendHTTPPost(postData);

    postData = "gateway_number=" + String(GATEWAY_NUMBER) +
               "&buoy_number=2&pitch=" + String(accPitch2) +
               "&roll=" + String(accRoll2) +
               "&tegangan=" + String(tegangan2) +
               "&suhu=" + String(suhu2) +
               "&humidity=" + String(humidity2);
    sendHTTPPost(postData);

    counter = 0;
    buoyStatus = false;
  } else {
    Serial.println("WiFi Disconnected! Trying to reconnect...");
    connectWiFi();  // Reconnect WiFi if disconnected
  }
}

void sendHTTPPost(String postData) {
  HTTPClient http;
  WiFiClient client;

  http.setTimeout(1000); // Set timeout 5 seconds for HTTP request
  http.begin(client, serverURL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  int httpCode = http.POST(postData);
  
  Serial.print("HTTP POST Code: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    String payload = http.getString();
    Serial.print("Response payload: ");
    Serial.println(payload);
  } else {
    Serial.println("Error on HTTP request");
  }

  http.end();
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
}
