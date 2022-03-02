
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
import json
import threading
import time
import dronekit as dk
import logging
import sys
import logging
import os as os
import geopy
# import geopy.dis
import haversine as hs
from queue import Queue

alpha = 100
Beta = 1.25
gama = 4
delt = 15



#nvrt and nrt are both numpy arrays
def f_att(x,nrt,nvr,nvt):
    nvrt = (nvr-nvt)/abs(np.linalg.norm(nvr-nvt))
    first  = (alpha/(Beta**2)) - ((alpha)/(x+Beta)**2)
    second = 2*gama*abs(np.linalg.norm(nvr-nvt))

    f = first*nrt + second*nvrt

    return f;

def f_rep(x,nro,vr,vobs,):
    nvro = (vr-vobs)/abs(np.linalg.norm(vr-vobs))
    first = -delt*(x-d0)*((x-d1)**2)
    second = rho*np.linalg.norm(vr-vobs)/amax

    if(x<=d1):
        rho = 0.50
        f = first*nro + second*nvro
    else:
        f = second*nvro
    return f

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

def calc_newgeo1(latitude,longitude,dist,bear=90):
    start = geopy.Point(latitude,longitude)
    # for i in range(50):
    d = geopy.distance.distance(kilometers=dist) # distance to be travelled

    # Use the `destination` method with a bearing of 0 degrees (which is north)
    # in order to go from point `start` 1 km to north.
    final = d.destination(point=start, bearing=bear).format_decimal()
    print(str(final))
    return final

def cdnt_to_bearing(ls):
    res = []
    x1 = ls[0][0]
    y1 = ls[0][1]
    for i in range(1,len(ls)):
        bear = atan2(ls[i][0],ls[i][1])
        bear  =degrees(bear)
        bear =90-bear
        disti = sqrt(ls[i][0]**2 + ls[i][1]**2)

        res.append([bear,disti])

    return res



# this function returns the distance between two coordinates(geo) in meters
def calc_dist_geo(lat1,lon1,lat2,lon2):
    loc1 = (lat1,lon1)
    loc2  = (lat2,lon2)
    return (hs.haversine(loc1, loc2)*1000)  #hs.haversine return in kilo meters

def get_bearing(lat1, long1, lat2, long2):
    dLon = (long2 - long1)
    x = cos(radians(lat2)) * sin(radians(dLon))
    y = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(dLon))
    brng = np.arctan2(x,y) #https://www.igismap.com/what-is-bearing-angle-and-calculate-between-two-points/
    brng = np.degrees(brng)
    return brng
# 1:procedureDAPF
#     2:buildthediscreteconfigurationspace
#     3:initializen,m,startcell(S),goalcell(G)
#     4:markcellsoccupiedbyobstacles
#     5:assignweightstocellsaccordingtoEquation(1)
#     6:currentcell=startcell
#     7:whilecurrentcellisnotequaltogoalcelldo
#         8:N←thesetofcurrentcellneighboringcells
#         9:choosenextcell=min(neighboringcell∈N)
#         10:savethecurrentpath
#         11:assignavalueof∞tonextcell
#         12:currentcell=nextcell
#     13:endwhile
#     14:pathoptimization
#     15:L←lengthofthepath
#     16:returncollision-freepath
# 17:endprocedure

# for instance -I22 (uav number 23 ) the udp outs are : -
# 127.0.0.1:14770
# 127.0.0.1:14771
if __name__ == '__main__':

    dno = int(input("Enter the number of uavs you want "))
    dronelist =[] # a list to store swarmbot objects
    # IP_list  = ['127.0.0.1:14550','127.0.0.1:14560','127.0.0.1:14570']
    ip_list = []
    for i in range(dno):
        command_input  = "cd ~/ardupilot/ArduCopter; sim_vehicle.py -I"+str(i)+" -L T"+str(i)+" --sysid "+str(i+1) # this command starts sitl instances
        os.system(command_input) # executes the above command(s) in seperate termnial always
        ip_list.append("127.0.0.1:"+ str(14550+(i*10)))


    # uav = SwarmBot(self.self_connection_string,self.sysid)
    j=1
    for i in IP_list:
        dronelist.append(SwarmBot(i,j))
        j+=1
    j=0

    # to do :- store the locations in files

    # keep North as (+ve )x or East as (+ve )x or vice versa but never put an arbitary direction as
    # north
    # the obstacle is a uav in less than D meter radius (obstacle need not to be only neibhouring uavs)
    # cross check if bearing can be used to calculate the vectors
    queue_list =[] # implement deque instead of queue
    for i in range(len(dronelist)):
        queue_list.append(Queue)

    for i in range(len(dronelist)):
        for j in range(5):
            queue_list[i].put(dronelist[i].get_pos())
# generate waypoint given the waypoint of first node (in formation)

    # put this in diff thread
    # T0=-35.36325518994991,149.16520406169866,585,90
    # T1=-35.363255189799645,149.16531409339734,585,0
    # T2=-35.36325518954919,149.165424125096,585,0
    # T3=-35.36325518919857,149.16553415679465,585,0
    # T4=-35.363255188747765,149.16564418849333,585,0
    # T5=-35.36325518819678,149.16575422019199,585,0
    # T6=-35.36325518754562,149.16586425189064,585,0
    # T7=-35.36325518679427,149.1659742835893,585,0
    # T8=-35.363255185942755,149.16608431528795,585,0
    # T9=-35.363255184991054,149.1661943469866,585,0
    # while True:
