import dronekit
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
import pickle
import threading
import time
import dronekit as dk
import logging
import sys
import logging
import os as os
import geopy
import subprocess
# import geopy.dis
import haversine as hs
import csv

kobsti = 649.152
alpha = 100
Beta = 1.25
gama = 1.1
delt = 15
rho = 0.50
d0 =0
d1 = 0
amax = 3
beta = Beta
intit_flag =0
# north is y
def f_att(x,nrt,nvr,nvt):
    nvrt = (nvr-nvt)/abs(np.linalg.norm(nvr-nvt))
    first  = (alpha/(Beta**2)) - ((alpha)/(x+Beta)**2)
    second = 2*gama*abs(np.linalg.norm(nvr-nvt))

    f = first*nrt + second*nvrt

    return f;
# Function to find the line given two points
def lineFromPoints(P, Q, a, b, c):
    a = Q[1] - P[1]
    b = P[0] - Q[0]
    c = a * (P[0]) + b * (P[1])
    return a, b, c

# Function which converts the input line to its
# perpendicular bisector. It also inputs the points
# whose mid-point lies on the bisector
def perpendicularBisectorFromLine(P, Q, a, b, c):
    mid_point = [(P[0] + Q[0])/2, (P[1] + Q[1])/2]

    # c = -bx + ay
    c = -b * (mid_point[0]) + a * (mid_point[1])
    temp = a
    a = -b
    b = temp
    return a, b, c

# Returns the intersection point of two lines
def lineLineIntersection(a1, b1, c1, a2, b2, c2):
    determinant = a1 * b2 - a2 * b1
    if (determinant == 0):

        # The lines are parallel. This is simplified
        # by returning a pair of (10.0)**19
        return [(10.0)**19, (10.0)**19]
    else:
        x = (b2 * c1 - b1 * c2)/determinant
        y = (a1 * c2 - a2 * c1)/determinant
        return [x, y]

# in most of the cased 3 coordinates are passed
# working function to find
# the centre of given geo coordinates
def GetCenterFromDegrees(lat1,lon1):
    if (len(lat1) <= 0):
        return false;

    num_coords = len(lat1)
    X = 0.0
    Y = 0.0
    Z = 0.0

    for i in range (len(lat1)):
        lat = lat1[i] * np.pi / 180
        lon = lon1[i] * np.pi / 180

        a = np.cos(lat) * np.cos(lon)
        b = np.cos(lat) * np.sin(lon)
        c = np.sin(lat);

        X += a
        Y += b
        Z += c

    X /= num_coords
    Y /= num_coords
    Z /= num_coords

    lon = np.arctan2(Y, X)
    hyp = np.sqrt(X * X + Y * Y)
    lat = np.arctan2(Z, hyp)

    newX = (lat * 180 / np.pi)
    newY = (lon * 180 / np.pi)
    return newX, newY

def findCircumCenter(P, Q, R):

    # Line PQ is represented as ax + by = c
    a, b, c = 0.0, 0.0, 0.0
    a, b, c = lineFromPoints(P, Q, a, b, c)

    # Line QR is represented as ex + fy = g
    e, f, g = 0.0, 0.0, 0.0
    e, f, g = lineFromPoints(Q, R, e, f, g)

    # Converting lines PQ and QR to perpendicular
    # vbisectors. After this, L = ax + by = c
    # M = ex + fy = g
    a, b, c = perpendicularBisectorFromLine(P, Q, a, b, c)
    e, f, g = perpendicularBisectorFromLine(Q, R, e, f, g)

    # The point of intersection of L and M gives
    # the circumcenter
    circumcenter = lineLineIntersection(a, b, c, e, f, g)

    if (circumcenter[0] == (10.0)**19 and circumcenter[1] == (10.0)**19):
        print("The two perpendicular bisectors found come parallel")
        print("Thus, the given points do not form a triangle and are collinear")

    else:
        print("The circumcenter of the triangle PQR is: ", end="")
        print("(", circumcenter[0], ",", circumcenter[1], ")")
        return [circumcenter[0],circumcenter[1]]
    return 0
def f_rep(x,nro,vr,vobs,):
    nvro = (vr-vobs)/abs(np.linalg.norm(vr-vobs))
    first = -delt*(x-d0)*((x-d1)**2)
    second = rho*np.linalg.norm(vr-vobs)/amax

    if(x<=d1):
        f = first*nro + second*nvro
    else:
        f = second*nvro
    return f

# the attraction function mentioned in
# A novel potential filed method for path planning of mobile robots by adapting animal motiol attributes
# https://sci-hub.ru/https://www.sciencedirect.com/science/article/abs/pii/S0921889016302159

def u_att(x,vr,vt):
    beta = 3.6
    alpha =50
    vr = np.asarray(vr[0:2])
    vt = np.asarray(vt)
    diff = vr-vt
    mag =( np.linalg.norm(diff))**2
    U = (alpha/beta**2)*x + alpha/(x+beta) - alpha/beta + gama*mag
    # v = sqrt(U)
    v = (U)**0.45
    return v
# /home/kartikey/Kartikey_swarm/geo_test.py
# calculates new new location from the given one
# given bearing(90 default ) and distance (in KM)
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
# this function aims to perform the trialteration to find the desired points
# given the distances from three known points
def trialter(LatA,LonA,DistA,LatB,LonB,DistB,LatC,LonC,DistC):
    #assuming elevation = 0
    earthR = 6371
    # LatA = 37.418436
    # LonA = -121.963477
    # DistA = 0.265710701754
    # LatB = 37.417243
    # LonB = -121.961889
    # DistB = 0.234592423446
    # LatC = 37.418692
    # LonC = -121.960194
    # DistC = 0.0548954278262

    #using authalic sphere
    #if using an ellipsoid this step is slightly different
    #Convert geodetic Lat/Long to ECEF xyz
    #   1. Convert Lat/Long to radians
    #   2. Convert Lat/Long(radians) to ECEF
    xA = earthR *(cos(radians(LatA)) * cos(radians(LonA)))
    yA = earthR *(cos(radians(LatA)) * sin(radians(LonA)))
    zA = earthR *(sin(radians(LatA)))

    xB = earthR *(cos(radians(LatB)) * cos(radians(LonB)))
    yB = earthR *(cos(radians(LatB)) * sin(radians(LonB)))
    zB = earthR *(sin(radians(LatB)))

    xC = earthR *(cos(radians(LatC)) * cos(radians(LonC)))
    yC = earthR *(cos(radians(LatC)) * sin(radians(LonC)))
    zC = earthR *(sin(radians(LatC)))

    P1 = np.array([xA, yA, zA])
    P2 = np.array([xB, yB, zB])
    P3 = np.array([xC, yC, zC])

    #from wikipedia
    #transform to get circle 1 at origin
    #transform to get circle 2 on x axis
    ex = (P2 - P1)/(np.linalg.norm(P2 - P1))
    i = np.dot(ex, P3 - P1)
    ey = (P3 - P1 - i*ex)/(np.linalg.norm(P3 - P1 - i*ex))
    ez = np.cross(ex,ey)
    d = np.linalg.norm(P2 - P1)
    j = np.dot(ey, P3 - P1)

    #from wikipedia
    #plug and chug using above values
    x = (pow(DistA,2) - pow(DistB,2) + pow(d,2))/(2*d)
    y = ((pow(DistA,2) - pow(DistC,2) + pow(i,2) + pow(j,2))/(2*j)) - ((i/j)*x)

    # only one case shown here
    z = np.sqrt(pow(DistA,2) - pow(x,2) - pow(y,2))

    #triPt is an array with ECEF x,y,z of trilateration point
    triPt = P1 + x*ex + y*ey + z*ez

    #convert back to lat/long from ECEF
    #convert to degrees
    lat = degrees(asin(triPt[2] / earthR))
    lon = degrees(atan2(triPt[1],triPt[0]))

    print( lat, lon)
    return (lat,lon)
# Generates the list of neibhours
# to be considered for formation maintaince and (collision avoidance temporarly)
def neighbour_list(dno,sysid):
    res = []
    if(dno%2==0):
        if(sysid!=1):
            res.append(sysid-1)
        if(sysid!=dno):
            res.append(sysid+1)
        res.append(dno-sysid)
    # if res[-1] == sysid:
    #     res.pop()
    else:
        if(sysid!=1):
            res.append(sysid-1)
        if(sysid!=dno):
            res.append(sysid+1)
        res.append(dno-sysid+1)
    if res[-1] == sysid:
        res.pop()
    i =1
    while(len(res)<3 and i<=dno):
        if(res.count(i) ==0 and i!=sysid):
            res.append(i)
        i+=1

    return res

def calc_newgeo(latitude,longitude,bear=90):
    start = geopy.Point(latitude,longitude)
    for i in range(50):
        d = geopy.distance.distance(kilometers=(0.01*(i+1)))

        # Use the `destination` method with a bearing of 0 degrees (which is north)
        # in order to go from point `start` 1 km to north.
        final = d.destination(point=start, bearing=bear).format_decimal()
        print("T"+str(i)+"="+str(final))
        return final

# this function returns the distance between two coordinates(geo) in meters
def calc_dist_geo(lat1,lon1,lat2,lon2):
    loc1 = (lat1,lon1)
    loc2  = (lat2,lon2)
    return (hs.haversine(loc1, loc2)*1000)  #hs.haversine return in kilo meters

#  calculates bearing and return in degrees
def get_bearing(lat1, long1, lat2, long2):
    dLon = (long2 - long1)
    x = cos(radians(lat2)) * sin(radians(dLon))
    y = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(dLon))
    brng = np.arctan2(x,y)
    brng = np.degrees(brng)

    return brng

# check if the UAV is armable
# used to make sure that the mission starts for all the UAvs at same points
def check_arm(dronelist):
    for i in dronelist:
        if ( not (i.vehicle.is_armable)):
            return 0
    return 1
# probally wrong for geo locatoin but finds the incentre for cartesian coodinates
def incentre_geo(lat1,lon1,lat2,lon2,lat3,lon3):

    x = (a * x1 + b * x2 + c * x3) / (a + b + c)
    y = (a * y1 + b * y2 + c * y3) / (a + b + c)
# trialter(LatA,LonA,DistA,LatB,LonB,DistB,LatC,LonC,DistC)

# this function aims to generate the formation velocity fot UAV to maintain the formation
def form_vel(dronelist,drn,sysid,neighbors,neighbor_dist,bear_dict,f1,bear_nxt):
    # neighbours se woh point nikalenge jaha UAV ko hona chahiyen thh
    # aur velocity ka difference bhi
    # FIRST WE have
    f1.write("fdsjsda\n")
    DistA = neighbor_dist[neighbors[0]]/1000
    DistB = neighbor_dist[neighbors[1]]/1000 # should be in kilometers
    DistC = neighbor_dist[neighbors[2]]/1000
    LatA,LonA = dronelist[neighbors[0]-1].get_pos()
    LatB,LonB = dronelist[neighbors[1]-1].get_pos()
    LatC,LonC = dronelist[neighbors[2]-1].get_pos()
    st = str("the neibhours are "+str((dronelist[neighbors[0]-1]).get_sys_id())+", "+str((dronelist[neighbors[1]-1]).get_sys_id())+", "+str((dronelist[neighbors[2]-1]).get_sys_id())+ "\n"  )
    st = (str("Neighbours corrds are :-\n") + str(LatA)+","+ str(LonA)+"<green> \n"+ str(LatB)+","+ str(LonB)+"<green>\n"+ str(LatC)+","+ str(LonC)+"<green>\n" )
    f1.write(str(st))
    clat ,clong = dronelist[i].get_pos()

    st = (str("Current corrds are :-\n") + str(clat)+","+ str(clong)+"<yellow>\n")
    f1.write(str(st))

    st  = (str(LatB)+","+str(LonB)+",red,circle\n")
    f1.write(str(st))

    tLatA,tLonA = calc_newgeo1(LatA,LonA,DistA,bear_dict[neighbors[0]])
    tLatB,tLonB = calc_newgeo1(LatB,LonB,DistB,bear_dict[neighbors[1]])
    tLatC,tLonC = calc_newgeo1(LatC,LonC,DistC,bear_dict[neighbors[2]])
    st = (str("after corrdinates are  :-\n") + str(tLatA)+","+ str(tLonA)+"<red> \n"+ str(tLatB)+","+ str(tLonB)+"<red>\n"+ str(tLatC)+","+ str(tLonC)+"<red>\n" )
    f1.write(str(st))
    st  = (str(tLatB)+","+str(tLonB)+",green,square\n")
    f1.write(str(st))

    #what if the bearing is in negative direction or positive (done)

    # Given distance and bearings find the 3 points in cartesian plane
    # and than find the circumcentre
    # P = findCircumCenter([tLatA,tLonA ],[tLatB,tLonB],[tLatC,tLonC])
    # # st = str("the avoid velocities for sysid "+ str(sysid)+" are "+str(avoid_nvel)+" "+str(avoid_evel)+"\n")
    # st  = (str(P[0])+","+str(P[1])+",red,circle\n")
    # f1.write(str(st))

    P= GetCenterFromDegrees([tLatA,tLatB,tLatC],[tLonA,tLonB,tLonC])
    st = (str("Final corrds are :-\n") + str(P[0])+","+ str(P[1])+"<yellow> \n")
    f1.write(str(st))
    # st  = (str(P1[0])+","+str(P1[1])+",green,square\n")
    # f1.write(str(st))

    # dlat,dlon = trialter(LatA,LonA,DistA,LatB,LonB,DistB,LatC,LonC,DistC)#here is an abiguity
    # what will happen if circle radius dont intersect
    dlat = P[0]

    dlon = P[1]
    dist  = calc_dist_geo(dlat,dlon,clat,clong) # (in meters)

    print("dist formation is ",str(dist))
    v_north = 0
    v_east = 0
    sign =1
    # dist<10
    #if
    v_mag = 0

    if(dist >50 or dist <15):

        pass
        # limit this so the func is not active always
    else :
        v1 = dronelist[neighbors[0]-1].get_vel2()
        v2 = dronelist[neighbors[1]-1].get_vel2()
        v3 = dronelist[neighbors[2]-1].get_vel2()
        vavg = [(v1[0]+v2[0]+v3[0])/3,(v1[1]+v2[1]+v3[1])/3]

        v_mag = u_att(dist,drn.get_vel2(),vavg)
        st  = (str("distance is ")+str(dist)+" and vmag is  "+str(v_mag)+"\n")
        f1.write(str(st))
        bear1 = get_bearing(clat,clong,dlat,dlon)
        if(abs(bear1-bear_nxt)>90):
            sign =-1
        # v_north = v_mag*cos((bear1*pi)/180)
        #
        # v_east = v_mag*sin((bear1*pi)/180)
        st  = (str("v_north is ")+str(v_north)+" and v_east is  "+str(v_east)+"\n")
        f1.write(str(st))

    return sign*v_mag # return [v_north,v_east]


def call_subprocess(i):


    command_input  = "sim_vehicle.py -I"+str(i)+" -L T"+str(i)+" --sysid "+str(i+1) # this command starts sitl instances
    # os.popen(command_input).read() # executes the above command(s) in seperate termnial always
    # using subprocess as os waits for process to finish
    process_list.append(subprocess.Popen(command_input,cwd  = "/home/kartikey/ardupilot/ArduCopter",shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT))
    print("command is "+command_input)
    return (("127.0.0.1:"+ str(14550+(i*10))))

def goto_intial_points(drn,lat,long,alti = 10):
    a_location = LocationGlobalRelative(lat,long, alti)
    drn.vehicle.airspeed = 10;
    while intit_flag ==0:
        drn.vehicle.simple_goto(a_location,10)
        time.sleep(1)


    # drn.vehicle.simple_goto(a_location,10)
    # print("goto for sysid "+str(drn.get_sys_id()))
    # time.sleep(0.55)
    # drn.vehicle.simple_goto(a_location,10)
    # time.sleep(0.5)
    # drn.vehicle.simple_goto(a_location,10)
    # time.sleep(0.5)
    #
    # drn.vehicle.simple_goto(a_location,10)
    # time.sleep(0.5)
    #
    # drn.vehicle.simple_goto(a_location,10)

# dyanmic potetial field method
def bugs(do,kobsti,maxu,vr,vt,closing):

    vr = np.asarray(vr)
    vt = np.asarray(vt)
    diff = vt-vr
    print("diff is " + str(diff))
    mag =( np.linalg.norm(diff))**2
    # if diff[0]<0 or diff[1]<0:
    #
    #     mag =0
    if(closing>0):
        mag =0
    print("mag is "+str(mag))

    dk = 8 # the distance for which the uav starts avoiding
    U = 0
    if(do>dk) : # change here
        pass
    else:
        U = 0.5*kobsti*((1/do - 1/dk)**1.7) + gama*mag

    print("U is ",str(U))

    return sqrt(U)

def reslove_vel_ned(vx,vy,heading):
    mag=  sqrt(vx**2 + vy**2)
    v_n = mag*cos((heading*pi)/180)
    v_e = mag*sin((heading*pi)/180)
    return (v_n,v_e)



def generate_avoid_velo(neighbor,dronelist,drn,maxu,kobsti):
    res = [0,0]

    for i in neighbor:
        clat,clong= drn.get_pos()
        dlat,dlong = dronelist[i-1].get_pos()
        do = calc_dist_geo(clat,clong,dlat,dlong)
        bear = get_bearing(clat, clong, dlat, dlong)
        vr = drn.get_vel2()
        dist_vec = [clat - dlat,clong - dlong]
        # vr = reslove_vel_ned(vr[0],vr[1],drn.heading())
        vt = dronelist[i-1].get_vel2()
        v_real_vec = [vr[0] - vt[0], vr[1] - vt[1]  ]

        dist_vec = np.asarray(dist_vec)
        v_real_vec = np.asarray(v_real_vec)

        closing = np.dot(dist_vec,v_real_vec)

        # vt =reslove_vel_ned(vt[0],vr[0],dronelist[i-1].heading())
        s_avoid = bugs(do,kobsti,maxu,vr,vt,closing)
        v_avoid = [0,0]
        if(s_avoid>0):
            v_avoid = [-s_avoid*cos((bear*pi)/180),-s_avoid*sin((bear*pi)/180)]  #north and east velocity

        res[0]+=v_avoid[0]
        res[1]+=v_avoid[1]

    return res



def follow_trajectories(sysid,drn,dronelist,start_time,cart_coord):

    f = open("/home/kartikey/Kartikey_swarm/drone_"+str(sysid)+".txt",'w+')
    file = open('/home/kartikey/Kartikey_swarm/traj'+str(sysid)+'.csv')
    f1 = open("/home/kartikey/Kartikey_swarm/drone_form_"+str(sysid)+".txt",'w+')
    csvreader = csv.reader(file)
    dno = len(dronelist)
    neighbour_list1= neighbour_list(dno,sysid)
    st = str("the neighbours for sysid "+ str(sysid)+" are "+str(neighbour_list1)+"\n")
    f.write(str(st))

    neighbor_dist_org = {} # make a map of original distanc which should be there

    for i in neighbour_list1:

        neighbor_dist_org[i] = sqrt((cart_coord[i-1][0] - cart_coord[sysid-1][0])**2 + (cart_coord[i-1][1] - cart_coord[sysid-1][1])**2)

    st = str("the neighbours distance dictionary for sysid "+ str(sysid)+" is \n"+str(neighbor_dist_org)+"\n")
    f.write(str(st))

    bear_org = {}
    for i in neighbour_list1:

        bear_org[i] = degrees(atan2((cart_coord[sysid-1][0]-cart_coord[i-1][0]),(cart_coord[sysid-1][1]-cart_coord[i-1][1])))
        # no need of below code as math.atan2 handles it itself
        # if((cart_coord[sysid-1][0]-cart_coord[i-1][0])<0):
        #     st = str(" changing bearing from "+str(bear_org[i])+str(" to ")+str(-1*bear_org[i]))
        #     f.write(str(st))
        #     bear_org[i]*=-1

    st = str("the neighbours bearing dictionary for sysid "+ str(sysid)+" is \n"+str(bear_org)+"\n")
    f.write(str(st))
    header = next(csvreader)
    print(header)
    flag =0
    rows = []
    for row in csvreader:
        rows.append(row)
    for i in range(len(rows)):
        for j in range(len(rows[i])):
            rows[i][j] = float(rows[i][j])

    print(rows)
    file.close()
    maxu = 12 # define maximum uav velocity here
    drn.vehicle.airspeed = 10;
    # a_location = LocationGlobalRelative(rows[0][2],rows[0][3], alti)
    # drn.vehicle.simple_goto(a_location,10)
    # time.sleep(20)
    time.sleep(2)
    # start_time = time.time()
    while(time.time()<start_time +15):
        time.sleep(0.2)
    # for i in range()
    print("starting to follow")
    print("############################")
    # if(flag == 0):
    #     t1 = 1.5
    #     if sysid ==1:
    #         t1+=0.3
    #
    #     flag =1
    #     time.sleep(t1)
    j=1
    while j<len(rows): # for tk in range(500)
        time.sleep(0.1)

        if(j>=len(rows)):
            print("####################")
            print("reached last waypoint")
            break
        dlat,dlong = rows[j][2],rows[j][3]
        clat ,clong = drn.get_pos()
        dist1  = calc_dist_geo(clat,clong,dlat,dlong)

        if (dist1<4):
            print(" #####*######### Updated J for sysid "+str(sysid)+" to "+str(j)+("###########"))
            j+=1
            continue
        t1 = start_time +rows[j][4] - time.time()
        if(t1<=0):

            u1 =maxu

        else:
            u1 = ((2*(dist1))/t1) - rows[j][5] + 0.2*(form_vel(dronelist,drn,sysid,neighbour_list1,neighbor_dist_org,bear_org,f1,bear1))
        if(u1>maxu):
            u1 = maxu
        bear1 = get_bearing(clat,clong,dlat,dlong)
        print("params for sysid "+str(sysid)+" "+str(clat)+" "+str(clong)+" "+str(dlat)+" "+str(dlong)+" bear:- "+str(bear1)+" dist:- "+str(dist1)+" u is "+str(u1))
        print("time is for sysid "+str(sysid)+ " is "+str(t1))
        nvel = u1*cos((bear1*pi)/180) # the inner bracket converts from radians to degrees
        evel = u1*sin((bear1*pi)/180)
        avoid_nvel=0
        avoid_evel=0
        kobsti = 69.152
        # if(drn.get_sys_id() == 1 ):

        avoid_nvel , avoid_evel = generate_avoid_velo(neighbour_list1,dronelist,drn,maxu,kobsti)
        print("the avoid velocities for sysid "+ str(sysid)+" are ",str(avoid_nvel)," ",str(avoid_evel))
        st = str("the avoid velocities for sysid "+ str(sysid)+" are "+str(avoid_nvel)+" "+str(avoid_evel)+"\n")
        f.write(str(st))

        # if(drn.get_sys_id() == 2 ):
        #
        #     avoid_nvel , avoid_evel = generate_avoid_velo([1,3,4],dronelist,drn,maxu,kobsti)
        #     print("the avoid velocities for sysid "+ str(sysid)+" are ",str(avoid_nvel)," ",str(avoid_evel))
        #     st = str("the avoid velocities for sysid "+ str(sysid)+" are "+str(avoid_nvel)+" "+str(avoid_evel)+"\n")
        #     f.write(str(st))

        # form_nvel,form_evel =0,0 # considered upwards
        # form_nvel,form_evel = form_vel(dronelist,drn,sysid,neighbour_list1,neighbor_dist_org,bear_org,f1,bear1)
        # the form_vel is added directly in the simulation
        print("the form velocities are", str(form_nvel)," ",str(form_evel))
        print("velocity is "+str(nvel)+" "+str(evel))
        print("avoid velocity for sysid " +str(sysid)+" is "+ str(avoid_nvel) + " , "+ str(avoid_evel))
        velocity_mav = drn.get_vel2()
        print("mav velocity is " , str(velocity_mav))
        print("the heading is ", str(drn.heading()))
        print("velocity for sysid " +str(sysid)+" obtainded is "+str(reslove_vel_ned(velocity_mav[0],velocity_mav[1],drn.heading())))
        nvel +=avoid_nvel
        evel += avoid_evel
        nvel+= form_nvel
        evel += form_evel
        drn.update_vel([nvel,evel,0])


        # st = ""
        # st = str(drn.get_pos())+" ,#333,circle"
        # print(st)

    print("the final value of j for sysid " +str(sysid)+ "  is " + str(j))
    drn.update_vel([0,0,0]) # stopping the drone at the end
    f.close()
def check_pos(dronelist,rows,thresh):
    for i in range(len(dronelist)):
        clat ,clong = dronelist[i].get_pos()
        dist = calc_dist_geo(rows[i][2],rows[i][3], clat, clong)
        if(dist>thresh):
            return 0
    return 1
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
    if(dno<1):
        print("invalid number of drones ")
        exit(0)
    dronelist =[] # a list to store swarmbot objects
    # call geo_test.py to generate the trajectory
    # IP_list  = ['127.0.0.1:14550','127.0.0.1:14560','127.0.0.1:14570']
    # Generating the trajectories
    command_input1  = "python3 geo_test.py " + str(dno) # this command starts sitl instances
    # os.popen(command_input).read() # executes the above command(s) in seperate termnial always
    # using subprocess as os waits for process to finish
    subprocess.Popen(command_input1,cwd  = "/home/kartikey/Kartikey_swarm",shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
    print("the command is " + str(command_input1))
    print("generating the trajectories ...")
    time.sleep(dno*5)
    with open('/home/kartikey/Kartikey_swarm/cart_coord.pkl', 'rb') as f:
        cart_coord = pickle.load(f)
    print("the cartesian coordinates list is  ")
    print(cart_coord) # cart_coord
    ip_list = []
    process_list =[]
    for i in range(dno):
        # command_input  = "sim_vehicle.py -I"+str(i)+" -L T"+str(i)+" --sysid "+str(i+1) # this command starts sitl instances
        # os.popen(command_input).read() # executes the above command(s) in seperate termnial always
        # using subprocess as os waits for process to finish
        # process_list.append(subprocess.Popen(command_input,cwd  = "/home/kartikey/ardupilot/ArduCopter",shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT))
        # print("command is "+command_input)
        # ip_list.append("127.0.0.1:"+ str(14550+(i*10)))
        ip_list.append(call_subprocess(i))
        time.sleep(9)

    print("End of dno loop")

    # uav = SwarmBot(self.self_connection_string,self.sysid)
    j=1
    for i in ip_list:
        dronelist.append(SwarmBot(i,j))
        j+=1
    j=0

    while(check_arm(dronelist) !=1):
        print("waiting for all to be intialized")
        time.sleep(1.5)

    print("######################################")
    print("########## ALL Intialize #########")
    print("######################################")

    # to do :- store the locations in files

    # keep North as (+ve )x or East as (+ve )x or vice versa but never put an arbitary direction as
    # north
    # the obstacle is a uav in less than D meter radius (obstacle need not to be only neibhouring uavs)
    # cross check if bearing can be used to calculate the vectors
    print("taking off now")
    print("the size of dronelist is "+str(len(dronelist)))
    thread_list_takeoff  = []
    alti  =10
    for i in dronelist:
        # i.arm_and_takeoff(10,1)
        thread_list_takeoff.append(threading.Thread(target=i.arm_and_takeoff,args=(alti,2,)))
        print("hellow")
    #wait here for all to
    c = 1
    for i in thread_list_takeoff:
        i.start()
        print("taking off sysid ",str(c) )
        c+=1
    vel = [5,10,0]
    time.sleep(3)
    print("hellow2")

    # while(check_arm(dronelist) !=1):
    #
    #     print("waiting for all to be intialized")
    #     time.sleep(1.5)

    # wait till all uav reach the common alti
    while(abs(dronelist[dno-1].get_alt()-alti) > 0.5 or abs(dronelist[0].get_alt()-alti)>0.5):
        print("current alti are ",str(dronelist[dno-1].get_alt())," ",str(dronelist[0].get_alt()))
        print("the difference is ",str(abs(dronelist[dno-1].get_alt()-alti)),str(abs(dronelist[0].get_alt()-alti)))

        print("reaching the altitude ")
        print("the current alti are ")
        for i in dronelist:
            print(" " ,str(i.get_alt())," \b")


        time.sleep(1)
    print("All has reached the altitude ")
    # go to the initial positions
    time.sleep(5)
    # for j in range(20):
    #     for i in dronelist:
    #         i.update_vel(vel)
    #         print("current velocity for sysid "+str(i.get_sys_id())+" is "+str(i.get_vel2()))
    #     print("updatedd vel")
    #     # the get vel 2 function is gives a list as follows:- (vel in y(north ) direction) , vel in x(easr direction), vel in down direction
    #     time.sleep(1)
    # reading the intial points
    print("Getting the intial points ")
    file = open('/home/kartikey/Kartikey_swarm/traj'+'_initial'+'.csv')
    csvreader = csv.reader(file)
    header = next(csvreader)
    # print(header)
    rows = []
    for row in csvreader:
        rows.append(row)
    for i in range(len(rows)):
        for j in range(len(rows[i])):
            rows[i][j] = float(rows[i][j])

    print(rows)
    # Getting to the intial point and waiting for ALL
    print("going to the initial positions")
    thread_list_init = []
    for i in range(len(dronelist)):
        thread_list_init.append(threading.Thread(target = goto_intial_points,args=(dronelist[i],rows[i][2],rows[i][3],alti,)))

        # thread_list_init[-1].start()
    for i in thread_list_init:
        i.start()
    # following the trajectories
    while (check_pos(dronelist,rows,2)!=1):
        # check for all drones if they have reached their initial points
        print("waiting for all the drones to reach the intial points ...")
        time.sleep(1)
    intit_flag  =1
    print("###############")
    print("following the trajectories now")
    thread_list_trajectory = []
    start_time = time.time()
    for i in range(len(dronelist)):
        thread_list_trajectory.append(threading.Thread(target=follow_trajectories,args=(i+1,dronelist[i],dronelist,start_time,cart_coord)))

    for i in thread_list_trajectory:
        i.start()
    print("Getting the finalpoints ")
    file = open('/home/kartikey/Kartikey_swarm/traj'+'_final'+'.csv')
    csvreader = csv.reader(file)
    header = next(csvreader)
    # print(header)
    rows = []
    for row in csvreader:
        rows.append(row)
    for i in range(len(rows)):
        for j in range(len(rows[i])):
            rows[i][j] = float(rows[i][j])

    while(check_pos(dronelist, rows,10)!=1):
        time.sleep(1)
        print("WAITING TO REACH THE FINAL WAYPOINTS")
    # time.sleep(80)
    print("8****** reached the final points ******8")
    time.sleep(5)
    #add a condition here to check if every node has reached the last

    # for i in range(5):
    #     for j in range(len(dronelist)):
    #         print("current velocity is "+str(dronelist[j].get_vel2()))
    #         print("its type is ",str(type(dronelist[j].get_vel2())))
    #         time.sleep(2)
    print("ending 123")
    for i in thread_list_takeoff:
        i.join()
    for i in dronelist:
        i.vehicle.close()
        # i.close()
    for i in process_list:
        i.terminate()
    for i in thread_list_trajectory:
        i.join()
    for i in thread_list_init:
        i.join()
    # t= float(input("enter 1 to exit and kill "))
    # dno = int(input("Enter 1 to killall"))
    time.sleep(5)
    print("exiting everything ")
    time.sleep(20)
    if(True):

        print("killing all \n")
        command_input  = "killall -9 xterm ; killall -9 mavproxy.py ; killall -9 python; killall -9 python3  "
        os.system(command_input)
# T0=-35.36235162,149.16357866,585,0
# T1=-35.36227662484164,149.16363969386836,585,0
# T2=-35.36220162965154,149.16370072762385,585,0
# T3=-35.362276624718355,149.1637617616051,585,0
# T4=-35.36235161975341,149.1638227956992,585,0
# T5=-35.36325518819678,149.16575422019199,585,0
# T6=-35.36325518754562,149.16586425189064,585,0
# T7=-35.36325518679427,149.1659742835893,585,0
# T8=-35.363255185942755,149.16608431528795,585,0
# T9=-35.363255184991054,149.1661943469866,585,0
    # file = open('~/Kartikey_swarm/traj1.csv')
    # csvreader = csv.reader(file)
# to do how to close subprocess
# how to run different terminal processes using subprocess
# how does a thread end




# can genrate a  path wiht dynamic formation control(of uavs )
