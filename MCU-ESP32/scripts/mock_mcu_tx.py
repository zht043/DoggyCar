#Mock for MCU_Interface Node Transmit Thread

# This script would need pyserial
#       pip install pyserial
import serial
from time import sleep


uart = serial.Serial('/dev/ttyUSB0', 115200, 8, serial.PARITY_ODD)
#print(uart.name)

# send connect request
uart.write(b'\x66')
if uart.read() == b'\x77':
    uart.write(b'\x88')
    

# packet_str = "Hi, ESP32\n"
# packet_data = bytes(packet_str, 'ascii')
# packet_data_length = len(bytearray(packet_data))
# packet_head = b'#' + (packet_data_length).to_bytes(1, byteorder='little')
# #print(packet_data_length)

# sleep(1)

# while True:
#     uart.write(packet_head + packet_data + b'\n')
#     #line = uart.readline()
#     #print(line.decode('ascii'), end='')
#     sleep(0.05)


uart.close()