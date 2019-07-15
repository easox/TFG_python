"""
 Simulation of a rotating 3D Cube
 Developed by Leonel Machava <leonelmachava@gmail.com>
 
http://codeNtronix.com
 
"""
import sys
import datetime
import numpy as np
import math, pygame,time,random
import sd_recieve



import uart_recieve,madgwickahrs,Quaternion_naive
from operator import itemgetter
from get_calibration_ellipsoid import calibration_ellipsoid

class Point3D:
  def __init__(self, x = 0, y = 0, z = 0):
    self.x, self.y, self.z = float(x), float(y), float(z)

  def rotateX(self, angle):
    """ Rotates the point around the X axis by the given angle in degrees. """
    rad = angle * math.pi / 180
    cosa = math.cos(rad)
    sina = math.sin(rad)
    y = self.y * cosa - self.z * sina
    z = self.y * sina + self.z * cosa
    return Point3D(self.x, y, z)

  def rotateY(self, angle):
    """ Rotates the point around the Y axis by the given angle in degrees. """
    rad = angle * math.pi / 180
    cosa = math.cos(rad)
    sina = math.sin(rad)
    z = self.z * cosa - self.x * sina
    x = self.z * sina + self.x * cosa
    return Point3D(x, self.y, z)

  def rotateZ(self, angle):
    """ Rotates the point around the Z axis by the given angle in degrees. """
    rad = angle * math.pi / 180
    cosa = math.cos(rad)
    sina = math.sin(rad)
    x = self.x * cosa - self.y * sina
    y = self.x * sina + self.y * cosa
    return Point3D(x, y, self.z)

  def project(self, win_width, win_height, fov, viewer_distance):
    """ Transforms this 3D point to 2D using a perspective projection. """
    factor = fov / (viewer_distance + self.z)
    x = self.x * factor + win_width / 2
    y = -self.y * factor + win_height / 2
    return Point3D(x, y, self.z)

class Simulation:
  def __init__(self, win_width = 640, win_height = 480):
    pygame.init()

    self.screen = pygame.display.set_mode((win_width, win_height))
    pygame.display.set_caption("Simulation of a rotating 3D Cube (http://codeNtronix.com)")

    self.clock = pygame.time.Clock()
    self.vertices = [Point3D(-1,1,-1),
                    Point3D(1,1,-1),
                    Point3D(1,-1,-1),
                    Point3D(-1,-1,-1),
                    Point3D(-1,1,1),
                    Point3D(1,1,1),
                    Point3D(1,-1,1),
                    Point3D(-1,-1,1)]

    # Define the vertices that compose each of the 6 faces. These numbers are
    # indices to the vertices list defined above.
    self.faces = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]

    # Define colors for each face
    self.colors = [(255,0,255),(255,0,0),(0,255,0),(0,0,255),(0,255,255),(255,255,0)]

    self.angle = 0
    self.roll=0
    self.yaw=0
    self.pitch=0

  def run(self,roll,pitch,yaw):
    """ Main Loop """
    reciever.ser.read(reciever.ser.in_waiting)
    while 1:
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          pygame.quit()
          sys.exit()

     
      self.screen.fill((0,32,0))
      self.roll,self.pitch,self.yaw=updateEuler()

# It will hold transformed vertices.
      t = []
      


      for v in self.vertices:
        # Rotate the point around X axis, then around Y axis, and finally around Z axis.
        r = v.rotateX(self.roll).rotateY(self.pitch).rotateZ(self.yaw)
        # Transform the point from 3D to 2D
        p = r.project(self.screen.get_width(), self.screen.get_height(), 256, 4)
        # Put the point in the list of transformed vertices
        t.append(p)

# Calculate the average Z values of each face.
      avg_z = []
      i = 0
      for f in self.faces:
        z = (t[f[0]].z + t[f[1]].z + t[f[2]].z + t[f[3]].z) / 4.0
        avg_z.append([i,z])
        i = i + 1

# Draw the faces using the Painter's algorithm:
# Distant faces are drawn before the closer ones.
      for tmp in sorted(avg_z,key=itemgetter(1),reverse=True):
        face_index = tmp[0]
        f = self.faces[face_index]
        pointlist = [(t[f[0]].x, t[f[0]].y), (t[f[1]].x, t[f[1]].y),
        (t[f[1]].x, t[f[1]].y), (t[f[2]].x, t[f[2]].y),
        (t[f[2]].x, t[f[2]].y), (t[f[3]].x, t[f[3]].y),
        (t[f[3]].x, t[f[3]].y), (t[f[0]].x, t[f[0]].y)]
        pygame.draw.polygon(self.screen,self.colors[face_index],pointlist)

      pygame.display.flip()



def updateEuler():

  data=reciever.read_packet()
  gx=data[3]-offset[0]
  gy=data[4]-offset[1]
  gz=data[5]-offset[2]
  if len(data)==6:

    mad_filter.update_imu([gx*math.pi/180,gy*math.pi/180,gz*math.pi/180],[data[0],data[1],data[2]])
  else:
    mx,my,mz=correct_magnetometer(np.array([[data[6]],[data[7]],[data[8]]]))
    mad_filter.update([gx*math.pi/180,gy*math.pi/180,gz*math.pi/180],[data[0],data[1],data[2]],[mx,my,mz])
    print(gx*math.pi/180,gy*math.pi/180,gz*math.pi/180)
    print(data[0],data[1],data[2])
    print(mx,my,mz)
    
  roll,pitch,yaw=quat_to_degree(mad_filter.quaternion)
  print(time.time())
  print()
  return roll,pitch,yaw

def quat_to_degree(q):
  return Quaternion_naive.getEulerAngles(q)
def correct_magnetometer(m):
  global cx,cy,cz
  
  return m[0]-cx, m[1]-cy,m[2]-cz
  
def calibration_gyro():
  calib_reciver=uart_recieve.uart_reciever()
  gx=0
  gy=0
  gz=0
  for i in range(0,100):
    data=calib_reciver.read_packet()
    gx+=data[3]
    gy+=data[4]
    gz+=data[5]

  return np.array([[gx/100],[gy/100],[gz/100]])

def calibration_mag():
  data = sd_recieve.read_mag_file("magcalib.txt")
  cx=0
  cy=0
  cz=0
  for item in data:
    cx+=item[0]
    cy+=item[1]
    cz+=item[2]
  cx=cx/len(data)
  cy=cy/len(data)
  cz=cz/len(data)

  return cx,cy,cz




if __name__ == "__main__":
  global cx,cy,cz

  global offset
  offset=calibration_gyro()
  cx,cy,cz=calibration_mag()
  mad_filter=madgwickahrs.MadgwickAHRS(1/50,beta=.1)
  reciever=uart_recieve.uart_reciever()
  cube=Simulation()
  cube.run(0,0,0)

 