#nvrt and nrt are both numpy arrays dronekit
from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from math import *
import threading
import random
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import LineString
from helper import *
import time
from SwarmBot import SwarmBot
import sys
import socket
import pickle
import json
import threading
import time
import dronekit as dk
import logging
import sys
import logging
import os as os
import geopy
import pandas as pd
# import geopy.dis
import haversine as hs
from queue import Queue
import matplotlib.pyplot as plt


alpha = 100
Beta = 1.25
gama = 4
delt = 15
rho = 0.50

def coord_list(dn,dis=10):
    # this function provide the x and y coordinates  for every drone locations
    ls =[]
    # dn is number of drones
    # for line y = 1.5x
    dn+=1
    for i in range(dn//2):
        x  = sqrt(((dis*i)**2)/(1+(1.5**2)))
        y = 1.5*x
        ls.append([x,y])

    # print("1st size is "+str(len(ls))
    print(len(ls))
    # for other line of the right angled triangle
    x1 = ls[-1][0]
    y1 = ls[-1][1]

    for i in range(1,(dn+1)//2):
        x  = x1 + sqrt(((dis*i)**2)/(1+(1.5**2)))
        y = -1.5*(x-x1)+y1
        ls.append([x,y])
    print(len(ls))
    return ls

def calc_newgeo(latitude,longitude,bear=90):
    start = geopy.Point(latitude,longitude)
    for i in range(50):
        d = geopy.distance.distance(kilometers=(0.01*(i+1))) # distance to be travelled

        # Use the `destination` method with a bearing of 0 degrees (which is north)
        # in order to go from point `start` 1 km to north.
        final = d.destination(point=start, bearing=90).format_decimal()
        print("T"+str(i)+"="+str(final))
        return final
# dist is in kM
def calc_newgeo1(latitude,longitude,dist,bear=90):
    start = geopy.Point(latitude,longitude)
    # for i in range(50):
    d = geopy.distance.distance(kilometers=dist) # distance to be travelled

    # Use the `destination` method with a bearing of 0 degrees (which is north)
    # in order to go from point `start` 1 km to north.
    final = d.destination(point=start, bearing=bear).format_decimal()

    final = final.split(',')
    for i in range(len(final)):
        final[i] = float(final[i])
    print(str(final))
    return final

def cdnt_to_bearing(ls):
    res = []
    x1 = ls[0][0]
    y1 = ls[0][1]
    for i in range(1,len(ls)):
        bear = atan2(ls[i][0],ls[i][1]) # dont forget that this is the angle with y axis and
        # x so x,y
        bear  =degrees(bear)
        # bear =90-bear
        disti = sqrt(ls[i][0]**2 + ls[i][1]**2)
        # if(ls[i][0]<0):
        #     bear*=-1

        res.append([bear,disti])

    return res
def get_bearing(lat1, long1, lat2, long2):
    dLon = (long2 - long1)
    x = cos(radians(lat2)) * sin(radians(dLon))
    y = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(dLon))
    brng = np.arctan2(x,y) #https://www.igismap.com/what-is-bearing-angle-and-calculate-between-two-points/
    brng = np.degrees(brng)
    return brng

def calc_path(lat1,long1,lat2,long2,dist):
    ls_5m = []
    bear  = get_bearing(lat1,long1,lat2,long2)
    clat1,clong1 = lat1,long1
    i =1
    while(calc_dist_geo(clat1,clong1,lat2,long2)>40):
        print("the distance is ",str(calc_dist_geo(clat1,clong1,lat2,long2)))
        ls_5m.append([clat1,clong1])
        bear  = get_bearing(clat1,clong1,lat2,long2)
        clat1,clong1  = calc_newgeo1(clat1,clong1,dist/1000,bear)
        i+=1
    ls_5m.append([clat1,clong1])

    return ls_5m


# this function returns the distance between two coordinates(geo) in meters
def calc_dist_geo(lat1,lon1,lat2,lon2):
    loc1 = (lat1,lon1)
    loc2  = (lat2,lon2)
    return (hs.haversine(loc1, loc2)*1000)  #hs.haversine return in kilo meters

# mavproxy.py --master=127.0.0.1:14551 --master=127.0.0.1:14561 --master=127.0.0.1:14571 --master=127.0.0.1:14581 --master=127.0.0.1:14591 --master=127.0.0.1:14601 --master=127.0.0.1:14611 --master=127.0.0.1:14621 --master=127.0.0.1:14631 --master=127.0.0.1:14641 --master=127.0.0.1:14651 --master=127.0.0.1:14661 --master=127.0.0.1:14671 --master=127.0.0.1:14681 --master=127.0.0.1:14691 --map
# lat1,long1 = -35.36215479, 149.16442275
# lat2,long2 = -35.36212384, 149.16527522
# dest = -35.360672 , 149.173438
lat1,long1 =  -35.36235162 ,149.16357866
lat2,long2 =  -35.36296747 ,149.16641315
bear = get_bearing(lat1,long1,lat2,long2)
print("the bearing is ",bear)
dist =calc_dist_geo(lat1,long1,lat2,long2) # in meters
print("dist is ",dist,"meters")

new_geo = calc_newgeo1(lat1, long1,dist/1000,bear) # giving in KM

print("the type of new geo 1 is ",str(type(new_geo)))

print("the new geo is ",str(new_geo))

# the main code starts here

# nd = int(input("Enter the number of drones "))
nd = int(sys.argv[1])
print("nd is "+str(nd))
# ls = coord_list(nd)
with open('/home/kartikey/Kartikey_swarm/cart_coord_cursor.pkl', 'rb') as f3:
    ls = pickle.load(f3)
# print(mynewlist)
# ls =mynewlist
dno = nd
print("the coordinate list (x and y plane ) is ")
print(ls)
with open('/home/kartikey/Kartikey_swarm/cart_coord.pkl', 'wb') as f:
    pickle.dump(ls, f)
f.close()

xs = []
xy  = []
for i in range(len(ls)):
    xs.append(ls[i][0])
    xy.append(ls[i][1])

nxs = np.asarray(xs)
nxy = np.asarray(xy)

plt.scatter(xs,xy,color="grey",marker="^")
# plt.show()
bear_dis_ls = []
bear_dis_ls.append([0,0])
bear_dis_ls.extend(cdnt_to_bearing(ls) )# gives bearing and distance
print("the length of coord_list is ",len(bear_dis_ls))
tk =[] # contains the  coordinates of the uavs
#generating geocoordinates
print("[bearing , distance] ")
print(bear_dis_ls)
#calc_newgeo1(latitude,longitude,dist,bear=90)
print("\n\n")
print("printing the new coordinates")
for i in bear_dis_ls:
    tk.append(calc_newgeo1(lat1, long1, i[1]/1000, i[0]))
    #                                   distance
print("the len of tk is ",str(len(tk)))

list_5m =calc_path(-35.36235162 ,149.16357866,-35.360672, 149.173438,20)
## the waypoints are at 20 m away form each other
print("path list is ")
print(list_5m)

trajectory1 = []

m=0
# adding time
# t = d/speed
# = 20/5
for t in list_5m:

    t.append(4*m)
    m+=1

# trajectory1.append(list_5m)
for i in range(dno):
    trajectory1.append([])

speed =5

for t in list_5m:
    for i in range(dno):
        trajectory1[i].append(calc_newgeo1(t[0],t[1],bear_dis_ls[i][1]/1000,bear_dis_ls[i][0]))
        trajectory1[i][-1].append(t[2])
        trajectory1[i][-1].append(speed)


k =1
for i in trajectory1:
    print("for drone ",str(k))
    print(i)
    k+=1
print("for plotting")

for i in trajectory1[1]:

    # for t in i:
    st = ""
    st = str(i[0])+","+str(i[1])+",#333,circle"
    print(st)

print("for the first drone")
flag=0
for i in trajectory1[0]:

    # for t in i:
    st = ""
    st = str(i[0])+","+str(i[1])+",red,circle"
    if(flag==0):
        flag+=1
        print(i)
    print(st)

pt_ls = []
lat_ls = []
long_ls = []
time_ls = []
time_ls = []
speed_ls = []
initial_points = []
final_points = []
tk =0
for k in trajectory1:
    tk+=1
    j=0
    pt_ls = []
    lat_ls = []
    long_ls = []
    time_ls = []
    time_ls = []
    speed_ls = []
    for i in k:
        j+=1
        pt_ls.append(j)
        lat_ls.append(i[0])
        long_ls.append(i[1])
        time_ls.append(i[2])
        speed_ls.append(i[3])


    initial_points.append([ pt_ls[0],lat_ls[0],long_ls[0],time_ls[0],speed_ls[0] ])
    final_points.append([ pt_ls[-1],lat_ls[-1],long_ls[-1],time_ls[-1],speed_ls[-1] ])

    dict_pd = {'Pt_No':pt_ls,'latitude':lat_ls,'longitude':long_ls,'time':time_ls,'speed':speed_ls}
    df = pd.DataFrame(dict_pd)

    df.to_csv('~/Kartikey_swarm/traj'+str(tk)+'.csv')

dict_pd1 = {'Pt_No':[row[0] for row in initial_points],'latitude':[row[1] for row in initial_points],'longitude':[row[2] for row in initial_points],'time':[row[3] for row in initial_points],'speed':[row[4] for row in initial_points]}
df1 = pd.DataFrame(dict_pd1)

df1.to_csv('~/Kartikey_swarm/traj'+'_initial'+'.csv')

dict_pd1 = {'Pt_No':[row[0] for row in final_points],'latitude':[row[1] for row in final_points],'longitude':[row[2] for row in final_points],'time':[row[3] for row in final_points],'speed':[row[4] for row in final_points]}
df1 = pd.DataFrame(dict_pd1)

df1.to_csv('~/Kartikey_swarm/traj'+'_final'+'.csv')
# print("the verifies distance is ",calc_dist_geo(trajectory1[0][3][0],trajectory1[0][3][1],trajectory1[2][3][0],trajectory1[2][3][1]))
# print("the original distance is ",bear_dis_ls[2])
plt.gca().set_aspect('equal',adjustable='box')

plt.show()
