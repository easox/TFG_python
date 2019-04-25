import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

X_TIME=2000

def plot_telemetry():
  global point
  global data 
  point=np.zeros((6,1))
  data= np.zeros((6,1))

  fig = plt.figure()
  accel=fig.add_subplot(2,1,1)
  gyro=fig.add_subplot(2,1,2)

  ax, = accel.plot([], [],lw=1,label='x')
  ay, = accel.plot([], [],lw=1,label='y')
  az, = accel.plot([], [],lw=1,label='z')

  gx, = gyro.plot([], [],lw=1,label='x')
  gy, = gyro.plot([], [],lw=1,label='y')
  gz, = gyro.plot([], [],lw=1,label='z')

  def animate(i):
    global data
    global point

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

    accel.set_xlim(np.amin(x), np.amax(x))
    ax.set_data(x,data[0])
    ay.set_data(x,data[1])
    az.set_data(x,data[2])

    gyro.set_xlim(np.amin(x), np.amax(x))
    gx.set_data(x,data[3])
    gy.set_data(x,data[4])
    gz.set_data(x,data[5])
    


  # Init only required for blitting to give a clean slate.
  def init():
  
    accel.grid(True)
    accel.margins(x=0)
    accel.legend()
    accel.set_ylim(-30,30)
    accel.title.set_text("Acceleration")
    accel.set_ylabel("acceleration (m^2/s)")
    accel.margins(x=0)
    accel.legend()

    gyro.grid(True)
    gyro.margins(x=0)
    gyro.legend()
    gyro.set_xlabel("Time (s)")
    gyro.set_ylabel("angular speed (º/s)")
    gyro.set_ylim(-30,30)
    gyro.title.set_text("Rotation")
    gyro.margins(x=0)
    gyro.legend()
    return ax, ay, az, gx, gy, gz
    
  ani = animation.FuncAnimation(fig, animate, init_func=init,
                                interval=1, blit=False)
  plt.show()


