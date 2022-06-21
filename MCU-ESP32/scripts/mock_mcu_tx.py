#Mock for MCU_Interface Node Transmit Thread

# This script would need pyserial
#       pip install pyserial
import serial
import struct
from time import sleep
from threading import Thread

# sudo chmod a+rw /dev/ttyUSB0

uart = serial.Serial('/dev/ttyUSB0', 115200, 8, serial.PARITY_ODD)
#print(uart.name)

# send connect request
uart.write(b'\x66')
if uart.read() == b'\x77':
    sleep(0.1)
    uart.write(b'\x88')


def prompt():
    while True:
        #print("xxx")
        sleep(1)

prompt_task = Thread(target=prompt)
prompt_task.start()


while True:
    steer_set_angle = 45
    steer_set_speed = 0.0
    esc_speed = 0.0
    led_cmd_byte = b'\xff'
    
    packet_data = b''
    packet_data += steer_set_angle.to_bytes(1, byteorder='little')
    packet_data += bytearray(struct.pack("ff",steer_set_speed, esc_speed)) 
    packet_data += led_cmd_byte
     
    packet_head = b'#' + len(packet_data).to_bytes(1, byteorder='little')
    print(packet_data)
    
    uart.write(packet_head + packet_data + b'\n')
    sleep(1.00 / 50)


prompt_task.join()
uart.close()