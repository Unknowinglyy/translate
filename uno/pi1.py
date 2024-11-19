import serial
import time

# Initialize serial communication with the Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Update 'ttyUSB0' as needed
time.sleep(2)  # Wait for the Arduino to initialize

# Function to send command to Arduino
def send_command(command):
    arduino.write(command.encode())  # Send command as a single character
    print(f"Sent command: {command}")

try:
    while True:
        # Example: Toggle LED on and off every 2 seconds
        send_command('1')  # Turn LED on
        time.sleep(2)
        send_command('0')  # Turn LED off
        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting program...")
    arduino.close()
