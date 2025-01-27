#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <BluetoothSerial.h> 

#define GATEWAY_NUMBER 2
#define NUM_ITERATIONS 10 
#define TIMEOUT 60000
#define FREQ 433E6
#define SPREADING_FACTOR 8
#define BANDWIDTH 41700 // 15,6 kHz
#define CODING_RATE 5  // Coding Rate 4/8
#define PREAMBLE_LENGTH 12

// SIM800L Serial Pins
#define SIM800_RX_PIN 16
#define SIM800_TX_PIN 17
#define SIM800_RST_PIN 4
SoftwareSerial SIM800(SIM800_RX_PIN, SIM800_TX_PIN);
BluetoothSerial SerialBT; 

#define USE_SSL true
#define DELAY_MS 500

// APN settings
const String APN = "byu"; // Ganti dengan pengaturan APN Anda
const String USER = "";    // GPRS User
const String PASS = "";    // GPRS Password

// Define the pins of LoRa module
const int csPin = 5;     // LoRa radio chip select
const int resetPin = 4;  // LoRa radio reset
const int irqPin = 25;   // Must be a hardware interrupt pin

const String FIREBASE_HOST = "https://gateway-data-gsm-default-rtdb.asia-southeast1.firebasedatabase.app/"; // Ganti dengan host Firebase
const String FIREBASE_SECRET = "7ydfdVT7E35GpD5J4Px4bHy9hPaesiPvfK8TSUTW"; // Ganti dengan secret Firebase

int RSSI;
float SNR;
unsigned long lastUpdateTime = 0;
bool zeroDataSent = false;
int counter = 0;
bool buoyStatus = false;

// Variables for buoy 1
int buoyNumber;
float accPitch = 0;
float accRoll = 0;
float tegangan = 0;
float suhu = 0;
float humidity = 0;
int light = 0;
double longitude = 0;
double latitude = 0;
String tanggal;
String waktu;
String Data;

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(115200);
  SerialBT.begin("Gateway-MaiBuoy");
  while (!Serial);

  // Inisialisasi komunikasi SIM800L
  SIM800.begin(9600);
  
  Serial.println("Menginisialisasi SIM800...");
  init_gsm();

  LoRa.setPins(csPin, resetPin, irqPin);
  Serial.println("LoRa Receiver Test");

  if (!LoRa.begin(FREQ)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(SPREADING_FACTOR); 
  LoRa.setSignalBandwidth(BANDWIDTH);  // Default 125 kHz
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setPreambleLength(PREAMBLE_LENGTH);
}

void loop() {
  updateBuoyData();
  
  // if (!is_gprs_connected()) {
  //   Serial.println("try to connect");
  //   gprs_connect();
  // }

  // if (buoyStatus) {
  //   post_to_firebase(Data);
  //   buoyStatus = false;
  //   counter = 0;
  // } 
  delay(1000); // Kirim data setiap 10 detik
}

// Fungsi untuk menghasilkan data dummy
void get_dummy_data(String data) {
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
  accPitch = data.substring(separator1 + 1, separator2).toFloat();
  accRoll = data.substring(separator2 + 1, separator3).toFloat();
  tegangan = data.substring(separator3 + 1, separator4).toFloat();
  suhu = data.substring(separator4 + 1, separator5).toFloat();
  humidity = data.substring(separator5 + 1, separator6).toFloat();
  light = data.substring(separator6 + 1, separator7).toInt();
  longitude = data.substring(separator7 + 1, separator8).toDouble();
  latitude = data.substring(separator8 + 1, separator9).toDouble();
  tanggal = data.substring(separator9 + 1, separator10);
  waktu = data.substring(separator10 + 1);

  // Format data JSON, pastikan setiap properti dipisahkan dengan benar
  Data = "{";
  Data += "\"BuoyNumber\":\"" + String(buoyNumber) + "\","; // Properti dengan tanda kutip
  Data += "\"AccPitch\":\"" + String(accPitch, 2) + "\",";   // Dua desimal untuk float
  Data += "\"AccRoll\":\"" + String(accRoll, 2) + "\",";     
  Data += "\"Tegangan\":\"" + String(tegangan, 2) + "\",";   
  Data += "\"Suhu\":\"" + String(suhu, 2) + "\",";           
  Data += "\"Humidity\":\"" + String(humidity, 2) + "\",";   
  Data += "\"Light\":\"" + String(light) + "\",";            
  Data += "\"Longitude\":\"" + String(longitude, 7) + "\","; 
  Data += "\"Latitude\":\"" + String(latitude, 7) + "\",";   
  Data += "\"Tanggal\":\"" + String(tanggal) + "\",";        
  Data += "\"Waktu\":\"" + String(waktu) + "\"";             
  Data += "}"; // Pastikan tidak ada koma di akhir objek JSON
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
      SerialBT.print("Received packet: ");
      SerialBT.println(receivedData);
      SerialBT.print("RSSI: ");
      SerialBT.println(RSSI);
      SerialBT.print("SNR: ");
      SerialBT.println(SNR);      

      get_dummy_data(receivedData);

      counter++;
    }
  }

  if (counter >= 1) {
    buoyStatus = true;
  }

  if (dataReceived) {
    lastUpdateTime = millis();
  }
}

void post_to_firebase(String data) {
  SIM800.println("AT+HTTPINIT");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);
  
  if (USE_SSL == true) {
    SIM800.println("AT+HTTPSSL=1");
    waitResponse("OK", 1000); // Tambahkan respons dan timeout
    delay(DELAY_MS);
  }

  SIM800.println("AT+HTTPPARA=\"CID\",1");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+HTTPPARA=\"URL\"," + FIREBASE_HOST + ".json?auth=" + FIREBASE_SECRET);
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+HTTPPARA=\"REDIR\",1");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+HTTPDATA=" + String(data.length()) + ",10000");
  waitResponse("DOWNLOAD", 1000); // Tambahkan respons dan timeout
  SIM800.println(data);
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+HTTPACTION=1");
  for (uint32_t start = millis(); millis() - start < 20000;) {
    while (!SIM800.available());
    String response = SIM800.readString();
    if (response.indexOf("+HTTPACTION:") > 0) {
      Serial.println(response);
      break;
    }
  }

  SIM800.println("AT+HTTPREAD");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+HTTPTERM");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);
}

void init_gsm() {
  SIM800.println("AT");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+CPIN?");
  waitResponse("+CPIN: READY", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+CFUN=1");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+CMEE=2");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+CBATCHK=1");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+CREG?");
  waitResponse("+CREG: 0,", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.print("AT+CMGF=1\r");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);
}

void gprs_connect() {
  SIM800.println("AT+SAPBR=0,1");
  waitResponse("OK", 60000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+SAPBR=3,1,\"APN\"," + APN);
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  if (USER != "") {
    SIM800.println("AT+SAPBR=3,1,\"USER\"," + USER);
    waitResponse("OK", 1000); // Tambahkan respons dan timeout
    delay(DELAY_MS);
  }

  if (PASS != "") {
    SIM800.println("AT+SAPBR=3,1,\"PASS\"," + PASS);
    waitResponse("OK", 1000); // Tambahkan respons dan timeout
    delay(DELAY_MS);
  }

  SIM800.println("AT+SAPBR=1,1");
  waitResponse("OK", 30000); // Tambahkan respons dan timeout
  delay(DELAY_MS);

  SIM800.println("AT+SAPBR=2,1");
  waitResponse("OK", 1000); // Tambahkan respons dan timeout
  delay(DELAY_MS);
}

boolean is_gprs_connected() {
  SIM800.println("AT+CGATT?");
  if (waitResponse("+CGATT: 1", 6000) == 1) {
    return false;
  }
  return true;
}

boolean waitResponse(String expected_answer, unsigned int timeout) {
  uint8_t answer = 0;
  String response;
  unsigned long previous = millis();

  while (SIM800.available() > 0) SIM800.read();

  do {
    if (SIM800.available() != 0) {
      char c = SIM800.read();
      response.concat(c);
      if (response.indexOf(expected_answer) > 0) {
        answer = 1;
      }
    }
  } while ((answer == 0) && ((millis() - previous) < timeout));

  Serial.println(response);
  return answer;
}
