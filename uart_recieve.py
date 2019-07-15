import os
import pty
import time
import struct
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from serial import Serial
from matplotlib import style

G4ACCEL=0.000122070312
GPS_BUFFER_SIZE=100
DPS500=0.015267175572
class uart_reciever():
    def __init__(device='/dev/tty.usbserial-A908578T', baudrate=115200):
        ser = Serial(device, baudrate, timeout=.1)
        time.sleep(1)
        init=0
        ser.read(ser.in_waiting)
        #ser.reset_input_buffer()
        print(ser.in_waiting)
    def read_packet(self):
        check_start()
        line=self.ser.read(17)
        timestamp,ax,ay,az,gx,gy,gz,end_byte=struct.unpack('<IhhhhhhB',line)
        
        if end_byte==0x71:
            line=self.ser.read(6)
            mag=struct.unpack('<hhh',line)

            ##gps_data=(ser.read(GPS_BUFFER_SIZE)).decode('utf-8','ignore')
            return ax,ay,az,gx*DPS500,gy*DPS500,gz*DPS500,mag[0],mag[1],mag[2]
        else:
            return ax,ay,az,gx*DPS500,gy*DPS500,gz*DPS500



    def check_start(self):
        while True:
                a=self.ser.read(1)
                if a==b'Y':
                    a=self.ser.read(1)
                    if a==b'Y':
                        break


    





