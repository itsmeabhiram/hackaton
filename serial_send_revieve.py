import serial
import time

# ESP32 Bluetooth COM Port
esp32_com_port = 'COM12'  # Replace with your ESP32's COM port
baud_rate = 115200

# Establish serial connection
ser = serial.Serial(esp32_com_port, baud_rate, timeout=1)  # Adjust baud rate as needed

def send_message(message):
    ser.write(message.encode())h
    print("Sent:", message)

def receive_message():
    if ser.in_waiting:
        received_message = ser.readline().decode().strip()
        print("Received:", received_message)

try:
    while True:
        user_input = input("Enter message to send (or type 'exit' to quit): ")
        if user_input.lower() == 'exit':
            break
        send_message(user_input)
        receive_message()

except KeyboardInterrupt:
    print("\nProgram terminated by user.")

finally:
    ser.close()
    print("Serial connection closed.")
