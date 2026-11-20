import serial
import threading

PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# Arduino resets when serial port opens
import time
time.sleep(2)

print("Connected.")
print("Type a command and press Enter.")
print("Type 'exit' to quit.\n")

def read_serial():
    while True:
        if ser.in_waiting:
            data = ser.readline().decode(errors="ignore").strip()
            if data:
                print(f"\nArduino: {data}")

thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

while True:
    command = input("> ")

    if command.lower() == "exit":
        break

    ser.write((command + "\n").encode())

ser.close()
