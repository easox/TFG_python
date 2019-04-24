import os
import pty
import time
import struct
import threading
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from serial import Serial
from matplotlib import style
from collections import deque

X_TIME=20


global point
global data 
point=np.zeros((6,1))
data= np.zeros((6,1))
#style.use('fivethirtyeight')
fig=plt.figure()
accel=fig.add_subplot(2,1,1)


gyro=fig.add_subplot(2,1,2)
gyro.title.set_text("Rotation")


def animate(i):
    print(time.time())
    global point
    global data 
    size=data.shape[1]
    point+=np.random.rand(6,1)-np.random.rand(6,1)
    #point+=np.ones((6,1))
    if size<X_TIME:   
      data=np.append(data,point,axis=1) 
      x=np.linspace(0,data.shape[1],data.shape[1])
   
    else:
        data=data[:,1:]
        data=np.append(data,point,axis=1) 
        x=np.linspace(i-17,i+3,data.shape[1])
    #x=np.linspace(0,len(data),50)
    #point=np.array([[1],[random.randrange(0,100)],[3],[4],[5],[6]])
    #data=np.append(data ,point,axis=1
    accel.clear()
    accel.set_ylim(-30,30)
    accel.grid(True)
    accel.title.set_text("Acceleration")
    accel.margins(x=0)
    accel.plot(x,data[0],lw=1,label='x')
    accel.plot(x,data[1],lw=1,label='y')
    accel.plot(x,data[2],lw=1,label='z')
    accel.legend()
    



ani=animation.FuncAnimation(fig,animate,frames=100)
plt.show()









