import paho.mqtt.client as mqtt
import requests

# MQTT Callback
def on_message(client, userdata, message):
    # Decode message payload
    payload = message.payload.decode()
    print(f"Received message: {payload}")

    # Parse payload
    data = payload.split(",")
    if len(data) == 13:
        gateway_number, pitch, roll, tegangan, suhu, humidity, light, longitude, latitude, tanggal, waktu, RSSI, SNR = data

        # Send data to PHP script
        response = requests.post("http://192.168.27.166/buoy/mqtt.php", data={
            'gateway_number': gateway_number,
            'buoy_number': message.topic.split("/")[-1],  # Assuming the buoy number is in the topic
            'pitch': pitch,
            'roll': roll,
            'tegangan': tegangan,
            'suhu': suhu,
            'humidity': humidity,
            'light' : light,
            'longitude' : longitude,
            'latitude' : latitude,
            'tanggal' : tanggal,
            'waktu' : waktu,
            'RSSI' : RSSI,
            'SNR' : SNR
        })
        print(f"Response from server: {response.text}")

# MQTT Setup
broker = "192.168.27.166"  # Ganti dengan IP broker MQTT Anda
port = 1883
client = mqtt.Client()
client.on_message = on_message

client.connect(broker, port)
client.subscribe("buoy/#")  # Subscribe to all buoy topics

client.loop_forever()
