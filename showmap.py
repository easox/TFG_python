import matplotlib.pyplot as plt
import math 
import numpy as np
from mpl_toolkits.basemap import Basemap

#GPS class able to convert lat and lon to meters

def coord_to_meters(self,lat,lon):
  x=lon*cos(lat)
  y=lat
  return x,y


class GPS_handler_class:

  #inits a subplot of the pre defined gui figure
  #inits a Basemap class to be able to project coordinates to meters

  def __init__(self):
    
    self.map_proj= Basemap(projection='mill',
                resolution='c',llcrnrlat=42,urcrnrlat=43,
                llcrnrlon=-84,urcrnrlon=-83)
  
  def coord_to_meters(self,lat,lon):
    x_aux,y_aux = self.map_proj(lon,lat)
    return x_aux,y_aux


    

if __name__=="__main__":
  global map_proj 
  global x_location,y_location
  a=1
  gui=plt.figure()
  gps_handler=GPS_handler_class(gui)
 
 