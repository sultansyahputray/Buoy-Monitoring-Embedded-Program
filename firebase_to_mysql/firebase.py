import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import mysql.connector

# Inisialisasi Firebase
cred = credentials.Certificate('gateway-data-gsm.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://gateway-data-gsm-default-rtdb.asia-southeast1.firebasedatabase.app/'
})

# Koneksi ke MySQL
mydb = mysql.connector.connect(
    host="localhost",
    user="root",  # Ganti dengan username MySQL Anda
    password="",  # Ganti dengan password MySQL Anda
    database="buoy_db",
)

mycursor = mydb.cursor()

# Fungsi untuk memasukkan data ke MySQL
def insert_data_to_mysql(value):
    try:
        # Ambil data dari dictionary, gunakan default jika tidak ada
        gateway_number = 0  # Default nilai jika tidak tersedia
        buoy_number = int(value.get('buoy_number', 0))
        acc_pitch = float(value.get('pitch', 0))
        acc_roll = float(value.get('roll', 0))
        voltage = float(value.get('voltage', 0))
        suhu = float(value.get('temperature', 0))
        humidity = float(value.get('humidity', 0))
        light = int(value.get('light', 0))
        longitude = float(value.get('longitude', 0))
        latitude = float(value.get('latitude', 0))
        tanggal = value.get('date', 'Not valid')
        waktu = value.get('time', 'Not valid')
        rssi = int(value.get('RSSI', 0))
        snr = float(value.get('SNR', 0))

        # Masukkan data ke dalam MySQL
        sql = """
        INSERT INTO buoys (
            gateway_number, buoy_number, pitch, roll, tegangan, suhu, humidity,
            cahaya, longitude, latitude, tanggal, waktu, rssi, snr, timestamp, updated_at
        ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, NOW(), NOW())
        """
        val = (gateway_number, buoy_number, acc_pitch, acc_roll, voltage, suhu, humidity, light, longitude, latitude, tanggal, waktu, rssi, snr)

        # Eksekusi statement
        mycursor.execute(sql, val)
        mydb.commit()
        print("New record inserted.")
    except mysql.connector.Error as e:
        print(f"MySQL Error: {e}")
    except Exception as e:
        print(f"Error inserting data to MySQL: {e}")

# Fungsi untuk mendeteksi penambahan data baru (child_added)
def listener(event, buoy_name):
    try:
        print(f"New child added in {buoy_name}: {event.data}")
        
        # Cek apakah event.data adalah dictionary
        if isinstance(event.data, dict):
            print(f"Inserting data from {buoy_name}: {event.data}")
            insert_data_to_mysql(event.data)
        else:
            print(f"Invalid data format for {buoy_name}. Skipping.")
    except Exception as e:
        print(f"Error in listener for {buoy_name}: {e}")

# Mendaftarkan listener untuk memantau penambahan data baru di setiap buoy
def setup_listeners():
    # Listener untuk buoy1
    ref_buoy1 = db.reference('/data/buoy1')
    ref_buoy1.listen(lambda event: listener(event, "Buoy 1"))

    # Listener untuk buoy2
    ref_buoy2 = db.reference('/data/buoy2')
    ref_buoy2.listen(lambda event: listener(event, "Buoy 2"))

# Menjaga agar program terus berjalan dan mendengarkan pembaruan real-time
try:
    print("Listening for new entries on /data/buoy1 and /data/buoy2...")
    setup_listeners()
    while True:
        pass  # Listener berjalan di background
except KeyboardInterrupt:
    print("Listener stopped.")
finally:
    mycursor.close()
    mydb.close()
