import os
import pty
import time
import sys
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
    line=in_file.read(17)
    check_eof(line,17)
      

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
        line=in_file.read(6)
        check_eof(line,6)
        mag=struct.unpack('<hhh',line)
        gps_line=in_file.read(GPS_BUFFER_SIZE)
        check_eof(gps_line,GPS_BUFFER_SIZE)
        gps_data=gps_line.decode('utf-8','ignore')
        


        print(mag[0])
        print(mag[1])
        print(mag[2])
        print(gps_data)
    line=in_file.read(4)
    check_eof(line,4)
    end_time=struct.unpack('<I',line)
    print(end_time[0])
    print('')
    print('')


def check_eof(string,read_b):
  if len(string)<read_b:
    in_file.close()
    print("Correct end")
    exit()

def check_start():
    while True:
            a=in_file.read(1) # if you only wanted to read 512 bytes, do .read(512)
            check_eof(a,1)

            if a==b'Y':
                a=in_file.read(1)
                check_eof(a,1)
                if a==b'Y':
                    break
            



    

in_file = open("out.txt", "rb") # opening for [r]eading as [b]inary

#in_file.close()

    
while True:
    read_packet()


