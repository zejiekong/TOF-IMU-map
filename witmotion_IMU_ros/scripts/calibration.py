#!/usr/bin/env python3

import serial
import struct
from time import sleep

if __name__ == "__main__":
    serial_port = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.1)
    
    serial_port.write(struct.pack("<BBBBB",0xFF,0xAA,0x69,0x88,0xB5))
    sleep(0.1)
    serial_port.write(struct.pack("<BBBBB",0xFF,0xAA,0x01,0x01,0x00))
    print("calibrating imu")
    sleep(5.0)
    print("calibration completed")
    serial_port.write(struct.pack("<BBBBB",0xFF,0xAA,0x01,0x00,0x00))
    sleep(0.1)
    print("saving config")
    serial_port.write(struct.pack("<BBBBB",0xFF,0xAA,0x00,0x00,0x00))
