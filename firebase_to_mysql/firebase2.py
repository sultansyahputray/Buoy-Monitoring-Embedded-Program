import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import mysql.connector
import time

# Inisialisasi Firebase
cred = credentials.Certificate('gateway-data-gsm-firebase-adminsdk-dgmh4-c3b2680782.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://gateway-data-gsm-default-rtdb.asia-southeast1.firebasedatabase.app/'
})

# Koneksi ke MySQL
mydb = mysql.connector.connect(
    host="localhost",
    user="root",  # Ganti dengan username MySQL Anda
    password="",  # Ganti dengan password MySQL Anda
    database="buoy_db"
)

mycursor = mydb.cursor()

# Referensi ke Firebase
ref = db.reference('/')

# Variabel untuk melacak kode unik terakhir
last_key = None

def fetch_and_store_new_data():
    global last_key  # Menggunakan variabel global untuk melacak key terakhir
    data = ref.get()
    if data:
        # Ambil key dan data terakhir
        latest_key = list(data.keys())[-1]
        latest_data = data[latest_key]

        # Hanya kirim data jika key berbeda dengan key terakhir
        if latest_key != last_key:
            last_key = latest_key  # Perbarui key terakhir
            gateway_number = 0
            buoy_number = latest_data.get('buoy_number', 0)  # Default ke 0 jika tidak ada
            acc_pitch = float(latest_data.get('pitch', 0))  # Default ke 0 jika tidak ada
            acc_roll = float(latest_data.get('roll', 0))  # Default ke 0 jika tidak ada
            voltage = float(latest_data.get('voltage', 0))  # Default ke 0 jika tidak ada
            suhu = float(latest_data.get('temperature', 0))  # Default ke 0 jika tidak ada
            humidity = float(latest_data.get('humidity', 0))  # Default ke 0 jika tidak ada
            light = float(latest_data.get('Cahaya', 0))  # Default ke 0 jika tidak ada
            longitude = float(latest_data.get('longitude', 0))  # Default ke 0 jika tidak ada
            latitude = float(latest_data.get('latitude', 0))  # Default ke 0 jika tidak ada
            tanggal = latest_data.get('date', 'Not valid')  # Default ke 'Not valid' jika tidak ada
            waktu = latest_data.get('time', 'Not valid')  # Default ke 'Not valid' jika tidak ada
            rssi = float(latest_data.get('RSSI', 0))  # Default ke 0 jika tidak ada
            snr = float(latest_data.get('SNR', 0))  # Default ke 0 jika tidak ada

            # Masukkan data ke dalam MySQL
            sql = """
                INSERT INTO buoys (
                    gateway_number, buoy_number, pitch, roll, tegangan, suhu, humidity, cahaya,
                    longitude, latitude, tanggal, waktu, rssi, snr, timestamp, updated_at
                )
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, NOW(), NOW())
            """
            val = (
                gateway_number, buoy_number, acc_pitch, acc_roll, voltage, suhu, humidity, light, 
                longitude, latitude, tanggal, waktu, rssi, snr
            )
            mycursor.execute(sql, val)

            mydb.commit()
            print(f"Data baru dengan key {latest_key} berhasil dikirim ke MySQL.")
        else:
            print("Tidak ada data baru untuk dikirim.")
    else:
        print("Firebase kosong atau tidak ada data.")

if __name__ == "__main__":
    try:
        print("Program mulai memantau Firebase untuk data baru.")
        while True:
            fetch_and_store_new_data()
            time.sleep(5)  # Tunggu 5 detik sebelum cek ulang
    except KeyboardInterrupt:
        print("Program dihentikan oleh pengguna.")
    finally:
        mycursor.close()
        mydb.close()
