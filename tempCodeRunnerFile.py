import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, accel = plt.subplots()

x = np.arange(0, 2*np.pi, 0.01)
ax, = accel.plot([], [],lw=1,label='x')
ay, = accel.plot([], [],lw=1,label='y')
az, = accel.plot([], [],lw=1,label='z')

def animate(i):
  if i>=1:
    
    ax.set_data(x,np.sin(x + i/10.0))  # update the data
    ay.set_data(x,np.sin(x + i/1.0))  # update the data
    #accel.autoscale(True,'x')
    print(time.time())
  else
    point=np.zeros((6,1))
    data= np.zeros((6,1))


  return ax, ay, az


# Init only required for blitting to give a clean slate.
def init():
 
  accel.grid(True)
  accel.margins(x=0)
  accel.legend()
  accel.set_ylim(-30,30)
  accel.title.set_text("Acceleration")
  accel.margins(x=0)
  accel.legend()
  return ax, ay, az
  
ani = animation.FuncAnimation(fig, animate, np.arange(1, 200), init_func=init,
                              interval=1, blit=True)
plt.show()