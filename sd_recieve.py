import os
import pty
import time
import pynmea2
import math
import showmap
import sys
import struct
import Quaternion_naive
import madgwickahrs
import allign_axis
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import random
from serial import Serial
from matplotlib import style

G4ACCEL=0.000122070312
GPS_BUFFER_SIZE=100
DPS500=0.015267175572

def gps_unpack(line):
  if line:
    msg=pynmea2.parse(line)
    if msg.is_valid:
      lat=msg.latitude
      lon=msg.longitude
      speed=float(msg.data[6])*1.852
      time=float(msg.data[0])
     


      return time,lon,lat,speed

    else:
      print("GPRMC data not valid")

      return 200,300,400,500
      
  else:
    print("Nothing to read")
    return 200,300,400,500


def read_mag_file(file="magcalib.txt"):
  data = np.empty((0,3), float) #x,y,z mag values
  in_file=open(file,"rb")
  line=in_file.read(2)
  if line==b'YY':
    while True:
      line=in_file.read(6)
      if check_eof(line,6):
        in_file.close()
        break

      mag=np.asarray(struct.unpack('<hhh',line))

      data=np.append(data,[mag],axis=0)
    return data
  

def read_data_file(file="out.txt"):
    data1 = np.empty((0,8), float) # timestamp, accelx,y,z,gyrox,y,z
    data2 = np.empty((0,8),float) #timestamp,gpstime,lon,lat,vel,track_angle,mag_var,magx,y,z
    in_file = open(file, "rb") # opening for [r]eading as [b]inary
    while True:
      if check_start(in_file):
        break
      
      line=in_file.read(17)
      if check_eof(line,17):
        in_file.close()
        break
        

      timestamp,ax,ay,az,gx,gy,gz,end_byte=struct.unpack('<IhhhhhhB',line)
      temp_data=np.array([[timestamp,ax*G4ACCEL,ay*G4ACCEL,az*G4ACCEL,gx*DPS500,gy*DPS500,gz*DPS500,end_byte]])
    
      
  
      data1=np.append(data1,temp_data,0)
      if end_byte==0x71:
          line=in_file.read(6)
          if check_eof(line,6):
            break
          mag=struct.unpack('<hhh',line)
          gps_line=in_file.read(GPS_BUFFER_SIZE)
          if check_eof(gps_line,GPS_BUFFER_SIZE):
            in_file.close()
            break
          gps_data=gps_line.decode('utf-8','ignore')
          gpstime,lon,lat,vel=gps_unpack(gps_data.split('\r')[0]) 
          temp_data=np.array([[timestamp,gpstime,lon,lat,vel,mag[0],mag[1],mag[2]]])
          
          data2=np.append(data2,temp_data,0)

         
          
      line=in_file.read(4)
      if check_eof(line,4):
        in_file.close()
        break

      end_time=struct.unpack('<I',line)
     
      

    return data1,data2


def check_eof(string,read_b):
  if len(string)<read_b:
    print("EOF file reached correctly")
    return 1
    
  else:
    return 0
    
    

def check_start(in_file):
    while True:
            a=in_file.read(1)
            if check_eof(a,1):
              in_file.close()
              return 1

            if a==b'Y':
                a=in_file.read(1)
                if check_eof(a,1):
                  in_file.close()
                  return 1
                if a==b'Y':
                    return 0
            



    
def display_all(data,data2):
  fig = plt.figure()
  fig2=plt.figure()
  fig3=plt.figure()
  fig4=plt.figure()
  fig5=plt.figure()

  accel=fig.add_subplot(3,1,1)
  accel.title.set_text("Acceleration")
  accel.set_ylabel("acceleration (m^2/s)")
  accel.set_xlabel("time (sec)")
  accel.margins(x=0)

  location=fig4.add_subplot(1,1,1)
  location.title.set_text("Location")
  location.set_ylabel("Position (m)")
 
  location.margins(x=0)

  gyro=fig.add_subplot(3,1,2)
  gyro.title.set_text("Gyro")
  gyro.set_ylabel("º/s")
  gyro.set_xlabel("time (sec)")
  gyro.margins(x=0)

  mag=fig.add_subplot(3,1,3)
  mag.title.set_text("Mag")
  mag.set_ylabel("Tesla")
  mag.set_xlabel("time (sec)")
  mag.margins(x=0)


  mag_calib=fig2.add_subplot(1,1,1)
  mag_calib.title.set_text("Magnetometer")
  mag_calib.set_ylabel("Teslas")
  mag_calib.margins(x=0)

  
  gps_speed=fig5.add_subplot(1,1,1)
  gps_speed.title.set_text("Speed")
  gps_speed.set_ylabel("km/hr")
  gps_speed.set_xlabel("time (sec)")
  gps_speed.margins(x=0)





  gps=fig3.add_subplot(1,1,1)
  gps.title.set_text("GPS")
  gps.set_ylabel("Location")
  gps.margins(x=0)
      
  mag_data=read_mag_file('magcalib.txt')
  mag_calib.plot(mag_data[:,0],lw=1,label='mx')
  mag_calib.plot(mag_data[:,1],lw=1,label='my')
  mag_calib.plot(mag_data[:,2],lw=1,label='mz')
  mag_calib.legend()



  #data,data2=read_data_file(file='out.txt')
  

  data2[1:,0]=data2[1:,0]/1000

    


  accel.plot(data[1:,0], data[1:,1],lw=1,label='ax')
  accel.plot(data[1:,0], data[1:,2],lw=1,label='ay')
  accel.plot(data[1:,0], data[1:,3],lw=1,label='az')



  gyro.plot(data[1:,0], data[1:,4],lw=1,label='gx')
  gyro.plot(data[1:,0], data[1:,5],lw=1,label='gy')
  gyro.plot(data[1:,0], data[1:,6],lw=1,label='gz')

  mag.plot(data2[1:,0], data2[1:,5],lw=1,label='mx')
  mag.plot(data2[1:,0], data2[1:,6],lw=1,label='my')
  mag.plot(data2[1:,0], data2[1:,7],lw=1,label='mz')


  location.plot(data2[1:,3],data2[1:,2],lw=1,label='xy location')
  location.set_aspect('equal')


  gps.plot(data2[1:,0], data2[1:,0],lw=1,label='time')
  gps.plot(data2[1:,0], data2[1:,1],lw=1,label='gpstime')
  gps.plot(data2[1:,0], data2[1:,2],lw=1,label='lon')
  gps.plot(data2[1:,0], data2[1:,3],lw=1,label='lat')
  gps_speed.plot(data2[1:,0], data2[1:,4],lw=1,label='vel')






  location.legend()
  accel.legend()
  gyro.legend()
  gps.legend()
  mag.legend()


  plt.show()

def coord_2_meters(data2):
  gps_handler=showmap.GPS_handler_class()
  for a in range(0,len(data2)):
  
    data2[a,3],data2[a,2]=gps_handler.coord_to_meters(data2[a,3],data2[a,2])

  return data2


def data_2_NWU(data1,data2):
  n=100
  m=20
  a_avg=avg_array(data1[:n,1:4],n)
  m_avg=avg_array(data2[:m,5:8],m)

  return allign_axis.allign_axis(a_avg,m_avg)

  
  

def calib_gyro(data):
  n=100
  g_avg=avg_array(data[:n,4:7],n)
  data[:,4:7]=data[:,4:7]-g_avg
  return data

def calibration_mag(data2):
  data = read_mag_file("magcalib.txt")
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

  data2[:,5:8]=data2[:,5:8]-np.array([cx,cy,cz])
  return data2


def main():
  
  mad_filter=madgwickahrs.MadgwickAHRS(1/50,beta=.1)

  data1,data2=read_data_file(file='out.txt')
  data1=calib_gyro(data1)
  data2=calibration_mag(data2)
  data2=coord_2_meters(data2)
  data2=center_meters(data2)
  
  R=data_2_NWU(data1,data2)
  g_NWU=np.matmul(R,np.transpose(data1[0,1:4]))
  

  orientation=np.empty((0,3),float)
  
  data2_index=0
  for a in range(0, len(data1)):
    if data1[a,7]==0:
      theta=updateEuler(data1[a,1:4],data1[a,4:7],data1[a,7],[0,0,0],mad_filter)
    else:
      theta=updateEuler(data1[a,1:4],data1[a,4:7],data1[a,7],data2[data2_index,5:8],mad_filter)
      data2_index+=1
    R_tot=allign_axis.eulerAnglesToRotationMatrix(theta)
    orientation=np.append(orientation,[theta],0)

  fig10=plt.figure()
  orient=fig10.add_subplot(1,1,1)
  orient.title.set_text("Orientation")
  orient.set_ylabel("rad")
  orient.set_xlabel("time (sec)")
  orient.margins(x=0)
  orient.plot(orientation[:,0],label="roll")
  orient.plot(orientation[:,1],label="pitch")
  orient.plot(orientation[:,2],label="yaw")
  orient.legend()
  display_all(data1,data2)


def center_meters(data):
  data[:,2]-=data[10,2]
  data[:,3]-=data[10,3]
  return data

def updateEuler(a,g,byte,m,mad_filter):

  
  if byte==0:

    mad_filter.update_imu([g[0]*math.pi/180,g[1]*math.pi/180,g[2]*math.pi/180],[a[0],a[1],a[2]])
  else:
    mad_filter.update([g[0]*math.pi/180,g[1]*math.pi/180,g[2]*math.pi/180],[a[0],a[1],a[2]],[m[0],m[1],m[2]])
   
    
  roll,pitch,yaw= Quaternion_naive.getEulerAngles(mad_filter.quaternion)
  
  
  return np.array([roll,pitch,yaw])*math.pi/180



def avg_array(data,n):

  avg_x=0
  avg_y=0
  avg_z=0

  for a in range(0,n):
    avg_x+=data[a,0]
    avg_y+=data[a,1]
    avg_z+=data[a,2]
    
  return np.array([avg_x,avg_y,avg_z])/n



main()