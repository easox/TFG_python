import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, accel = plt.subplots()

x = np.arange(0, 2*np.pi, 0.01)
line, = accel.plot([], [])

def animate(i):
    line.set_data(x,np.sin(x + i/10.0))  # update the data
    print(time.time())
    return line,


# Init only required for blitting to give a clean slate.
def init():
 
  accel.set_ylim(-1.1,1.1)
  accel.set_xlim(0,2*np.pi)
  #accel.axis('tight')
  accel.grid(True)
  accel.title.set_text("Acceleration")
  accel.margins(x=0)
  accel.legend()
  
  
  return line,

ani = animation.FuncAnimation(fig, animate, np.arange(1, 200), init_func=init,
                              interval=25, blit=True)
plt.show()