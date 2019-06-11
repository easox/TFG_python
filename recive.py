import os
import pty
import time
import struct
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import plot_telemetry as tel
from serial import Serial
from matplotlib import style

data=np.zeros((1,6))

#open a pySerial connection to the slave
ser = Serial('/dev/tty.SLAB_USBtoUART', 115200, timeout=5)

ser.flush()

tel.plot_telemetry()


