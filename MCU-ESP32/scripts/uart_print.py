#.........

# This script would need pyserial
#       pip install pyserial
import serial
from time import sleep


uart = serial.Serial('/dev/ttyUSB0', 115200, 8, serial.PARITY_ODD)
#print(uart.name)
while True:
    line = uart.readline()
    print(line.decode('ascii'), end='')


uart.close()