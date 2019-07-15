import math
import numpy as np

def normalize(x,y,z):
  denom=math.sqrt(x*x+y*y+z*z)
  x_norm=x/denom
  y_norm=y/denom
  z_norm=z/denom
  return np.array([x_norm,y_norm,z_norm])


# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R




def allign_axis(g,m):


  G=np.array([0,0,1]) ##z-axis gravity
  

  roll=math.atan2(g[1],g[2])
  pitch=math.atan2(-g[0],math.sqrt(g[1]*g[1]+g[2]*g[2]))
  yaw=math.atan2((math.sin(roll)*m[2]-math.cos(roll)*m[1]),
        (math.cos(pitch)*m[0]+math.sin(roll)*math.sin(pitch)*m[1]+math.cos(roll)*math.sin(pitch)*m[2]))

  return eulerAnglesToRotationMatrix([roll,pitch,yaw])






