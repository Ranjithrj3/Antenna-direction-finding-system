import serial
import time

ser = serial.Serial("COM5", 115200, timeout=1)

time.sleep(2)

print("Connected to ESP32")
print("Type an angle and press ENTER")
print()

while True:
    angle = input("Angle: ")

    print("You entered:", angle)

    ser.write((angle + "\n").encode())

    time.sleep(0.1)