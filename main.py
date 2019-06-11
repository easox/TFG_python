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

data=np.zeros((1,6))

#open a pySerial connection to the slave
ser = Serial('/dev/tty.usbserial-A908578T', 115200, timeout=1)
ser.flush()
i =0 
while 1:
    i+=1



    ser.write(random.randint(0,10))
    ser.write(random.randint(0,10))
    ser.write(random.randint(0,10))
    ser.write(random.randint(0,10))
    ser.write(random.randint(0,10))
    ser.write(random.randint(0,10))
    ser.write(bytes('\n','utf-8'))




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
    


