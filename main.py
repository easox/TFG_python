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
GPS_BUFFER_SIZE=200
DPS500=0.015267175572


def check_start():
    while True:
            a=ser.read(1)
            if a==b'Y':
                a=ser.read(1)
                if a==b'Y':
                    break


    


#open a pySerial connection to the slave
ser = Serial('/dev/tty.usbserial-A908578T', 115200, timeout=.1)
time.sleep(1)

f = open('data.txt', 'wb')

init=0
ser.read(ser.in_waiting)
#ser.reset_input_buffer()
print(ser.in_waiting)

    
while True:

    check_start()
    line=ser.read(17)
    num=struct.unpack('<IhhhhhhB',line)
    
    print(num[0])
    print(num[1]*G4ACCEL)
    print(num[2]*G4ACCEL)
    print(num[3]*G4ACCEL)
    print(num[4]*DPS500)
    print(num[5]*DPS500)
    print(num[6]*DPS500)
    print(num[7])
    print('')
    if num[7]==0x71:
        line=ser.read(6)
        mag=struct.unpack('<hhh',line)

        gps_data=(ser.read(GPS_BUFFER_SIZE)).decode('utf-8','ignore')
        line=ser.read(4)
        end_time=struct.unpack('<I',line)


        print(mag[0])
        print(mag[1])
        print(mag[2])
        print(gps_data)
        print(end_time)
        print('')
        print('')

    

        
    

   

    # try:

    #     line=line.decode('utf-8')
    #     print(line)
    #     f.write(line)
       


    # except:
    #     print(line)
        
    










# i=0

# while 1:
#     i+=1



#     ser.write(random.randint(0,10))
#     ser.write(random.randint(0,10))
#     ser.write(random.randint(0,10))
#     ser.write(random.randint(0,10))
#     ser.write(random.randint(0,10))
#     ser.write(random.randint(0,10))
#     ser.write(bytes('\n','utf-8'))




# line=ser.readline()
# point=line.split()
# data[:][0]=point

# while True:
#     line=ser.readline()
#     point=line.split()
#     data[:][i]=point
#     #for i in range(0,100):
#     print(data)  
#     plt.plot(data[1])
#     plt.show()
    


