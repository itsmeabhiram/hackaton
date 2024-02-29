import serial
import time

# ESP32 Bluetooth COM Port
esp32_com_port = 'COM12'  # Replace with your ESP32's COM port

# Establish serial connection
ser = serial.Serial(esp32_com_port, 115200, timeout=1)  # Adjust baud rate as needed

try:
    while True:
        # Read input from terminal
        data_to_send = input("Enter data to send: ")
        
        # Send data
        ser.write(data_to_send.encode() + b'\n')  # Sending data with newline for better readabilityf

        
        # Wait for 1 second
        time.sleep(1)

except KeyboardInterrupt:
    # Handle Ctrl+C to stop the loop
    print("KeyboardInterrupt: Stopping data transmission.")

finally:
    # Close connection
    ser.close()
