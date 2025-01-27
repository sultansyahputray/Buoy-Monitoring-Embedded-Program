#include "BluetoothSerial.h" 

BluetoothSerial SerialBT;

float accPitch;
float accRoll;
float vibrate;
float tegangan;
float suhu;
float humidity;
uint8_t cahaya;
uint8_t buoyNumber;
double longitude;
double latitude;
int RSSI;
float SNR;

String tanggal = "18/01/2025";

// Variabel untuk waktu
int hour = 16;     // Awal jam
int minute = 56;    // Awal menit
int second = 3;    // Awal detik

// Variabel untuk interval acak
unsigned long lastPrintTime = 0;  // Waktu terakhir data dikirim
unsigned long printInterval = 0; // Interval acak berikutnya

void setup() {
  Serial.begin(115200);
  SerialBT.begin("End_Node_1");
  while (!Serial);
  randomSeed(analogRead(0)); // Inisialisasi seed untuk random
  printInterval = random(13000, 17001); // Interval pertama antara 18 - 20 detik (ms)
}

void loop() {
  static unsigned long lastUpdateTime = millis(); // Waktu terakhir pembaruan waktu
  unsigned long currentTime = millis();

  // Perbarui waktu setiap detik
  if (currentTime - lastUpdateTime >= 1000) {
    lastUpdateTime = currentTime;
    updateTime();
  }

  // Kirim data setiap interval acak (18-20 detik)
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;
    printInterval = random(13000, 17001); // Tetapkan interval acak berikutnya

    // Generate random values within specified ranges
    accPitch = randomFloat(-30.0, 20.0);
    accRoll = randomFloat(-30.0, 20.0);
    tegangan = randomFloat(7.0, 7.4);
    suhu = randomFloat(25.0, 35.0);  // Example range for temperature
    humidity = randomFloat(60.0, 80.0);  // Example range for humidity
    cahaya = random(1000, 2550);  // Random light intensity (0-255)
    buoyNumber = 1;  // Example buoy number range
    longitude = randomFloat(-7.156212, -7.156202);  // Example longitude range
    latitude = randomFloat(112.782177, 112.782185);  // Example latitude range
    RSSI = random(-120, -118);
    SNR = randomFloatStep(-25.0, -20.0, 0.25);

    // Format waktu sebagai string
    String waktu = formatTime(hour, minute, second);

    // Create payload
    String payload = String(buoyNumber) + ";" + String(accPitch) + ";" + String(accRoll) + ";" + String(tegangan)
                   + ";" + String(suhu) + ";" + String(humidity) + ";" + String(cahaya) + ";" + String(longitude, 7)
                   + ";" + String(latitude, 7) + ";" + String(tanggal) + ";" + waktu;

    SerialBT.print("Received packet: ");
    SerialBT.println(payload);
    SerialBT.print("RSSI: ");
    SerialBT.println(RSSI);
    SerialBT.print("SNR: ");
    SerialBT.println(SNR);

    Serial.print("Data sent to Bluetooth at: ");
    Serial.println(waktu); // Debug di Serial Monitor
  }
}

float randomFloatStep(float min, float max, float step) {
  int range = (max - min) / step;  // Hitung jumlah langkah dalam rentang
  int randomStep = random(0, range + 1);  // Pilih langkah acak
  return min + (randomStep * step);  // Hitung nilai berdasarkan langkah
}

// Fungsi untuk menghasilkan nilai float random
float randomFloat(float min, float max) {
  return min + (float(random(0, 10000)) / 10000.0) * (max - min);
}

// Fungsi untuk memperbarui waktu
void updateTime() {
  second++;
  if (second >= 60) {
    second = 0;
    minute++;
    if (minute >= 60) {
      minute = 0;
      hour++;
      if (hour >= 24) {
        hour = 0;
      }
    }
  }
}

// Fungsi untuk memformat waktu sebagai string
String formatTime(int hour, int minute, int second) {
  char buffer[9];  // Format "hh:mm:ss" memerlukan 9 karakter termasuk null terminator
  sprintf(buffer, "%02d:%02d:%02d", hour, minute, second);
  return String(buffer);
}
