import os
import pty
import time
import struct
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from serial import Serial
from matplotlib import style

data=np.zeros((1,6))

#open a pySerial connection to the slave
ser = Serial('/dev/tty.usbserial-A908578T', 9600, timeout=1)
ser.flush()

line=ser.readline()
point=line.split()
data[:][0]=point

while True:
    line=ser.readline()
    point=line.split()
    data[:][i]=point
    #for i in range(0,100):
    print(data)  
    plt.plot(data[1])
    plt.show()
    


