import serial
import time

# ESP32 Bluetooth COM Port
esp32_com_port = 'COM12'  # Replace with your ESP32's COM port

# Establish serial connection
ser = serial.Serial(esp32_com_port, 115200)  # Adjust baud rate as needed

try:
    while True:
        # Send data
        ser.write(b"poda patti!\n")  # Sending data with newline for better readability
        
        # Wait for 1 second
        time.sleep(1)

except KeyboardInterrupt:
    # Handle Ctrl+C to stop the loop
    print("KeyboardInterrupt: Stopping data transmission.")

finally:
    # Close connection
    ser.close()
