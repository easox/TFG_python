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


in_file = open("write.txt", "rb") # opening for [r]eading as [b]inary
data = in_file.read(1) # if you only wanted to read 512 bytes, do .read(512)
print(data)
in_file.close()