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

ser = Serial('/dev/tty.usbserial-A908578T', 115200, timeout=10)
time.sleep(1)

f = open('data.txt', 'wb')
ser.reset_input_buffer()
while True:
  f.write(ser.read())


