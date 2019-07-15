#import libs
import showmap
import math as m
import parseGPS
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

#GLOBAL VARIABLES
global map_proj

x=np.array([])
y=np.array([])

map_img=mpimg.imread('map.png')


gps_handler=showmap.GPS_handler_class()
parser_gps=parseGPS.GPS_parser("out.txt")
lon,lat,speed,time=parser_gps.parseGPS_file()

itter=lon.shape
vel=np.array([])

#lat and long to meters
#velocity calculated as delta in distance
for a in range(0,lon.shape[0]):
  
    x_aux,y_aux=gps_handler.coord_to_meters(lat[a],lon[a])
    x=np.append(x,x_aux)
    y=np.append(y,y_aux)
    vel=np.append(vel,m.sqrt(x[a]**2+y[a]**2)-m.sqrt(x[a-1]**2+y[a-1]**2))

edges=np.array([[42.287124, -83.725693],
        [42.282886, -83.736331]])



# x_0,y_0=gps_handler.coord_to_meters(42.287124, -83.725693)
# x_1,y_1=gps_handler.coord_to_meters(42.282886, -83.736331)

x_0,y_0=gps_handler.coord_to_meters(edges[0][0],edges[0][1])
x_1,y_1=gps_handler.coord_to_meters(edges[1][0],edges[1][1])

edges_m=np.array([[x_0,y_0],[x_1,y_1]])




# x=x-x[0]
# y=y-y[0]
# edges_m[0]-=x[0]
# edges_m[1]-=y[1]
fig=plt.figure()
location=fig.add_subplot(2,1,1)
movement=fig.add_subplot(2,1,2)
location.plot(x,y)
location.imshow(map_img, extent=[np.amin(x),np.amax(x),np.amin(y),np.amax(y)])

location.scatter([np.amin(x),np.amax(x)],[np.amin(y),np.amax(y)],color='r')
#location.scatter(edges_m[1],edges_m[0],color='r')
#movement.plot(time,speed*10)
#location.plot(time,abs(vel*3.6))
#location.plot(time,abs(speed))
movement.plot(time,y)
movement.plot(time,x)


plt.show()

