# Importing Libraries
import serial
import time

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=10)


def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data


# while True:
#   inputValue = input("Enter input: ") # Taking input from user
#   value = write_read(inputValue)
#   print(value) # printing the value

while True:
    speed = 4.5
    steering = 30.0
    command = f'{speed:.1f} {steering:.1f}\r\n'
    print(command)
    arduino.write(bytes(command, 'utf-8'))
    time.sleep(1)
