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

def read_packet():
    check_start()
    line=ser.read(17)
    timestamp,ax,ay,az,gx,gy,gz,end_byte=struct.unpack('<IhhhhhhB',line)
    
    print(timestamp)
    print(ax*G4ACCEL)
    print(ay*G4ACCEL)
    print(az*G4ACCEL)
    print(gx*DPS500)
    print(gy*DPS500)
    print(gz*DPS500)
    print(end_byte)
    print('')
    if end_byte==0x71:
        line=ser.read(6)
        mag=struct.unpack('<hhh',line)

        gps_data=(ser.read(GPS_BUFFER_SIZE)).decode('utf-8','ignore')
        


        print(mag[0])
        print(mag[1])
        print(mag[2])
        print(gps_data)
    line=ser.read(4)
    end_time=struct.unpack('<I',line)
    print(end_time[0])
    print('')
    print('')



def check_start():
    while True:
            a=ser.read(1)
            if a==b'Y':
                a=ser.read(1)
                if a==b'Y':
                    break


    


#open a pySerial connection to the slave
ser = Serial('/dev/tty.usbserial-A908578T', 921600, timeout=.1)
time.sleep(1)

f = open('data.txt', 'wb')

init=0
ser.read(ser.in_waiting)
#ser.reset_input_buffer()
print(ser.in_waiting)

    
while True:
    read_packet()


