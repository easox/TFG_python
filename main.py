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

f = open('data.txt', 'w')

init=0
ser.read(ser.in_waiting)
#ser.reset_input_buffer()
print(ser.in_waiting)

    
while True:

    check_start()
    line=ser.read(11)
    num=struct.unpack('<IhhhB',line)
    
    print(num[0])
    print(num[1]*G4ACCEL)
    print(num[2]*G4ACCEL)
    print(num[3]*G4ACCEL)
    print(num[4])
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
    


