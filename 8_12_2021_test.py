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
# north is y
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
        f = first*nro + second*nvro
    else:
        f = second*nvro
    return f

def u_rep(x,vr,vt):

    vr = np.asarray(vr)
    vt = np.asarray(vt)
    diff = vr-vt
    mag =( np.linalg.norm(diff))**2
    U = (alpha/beta**2)*x + alpha/(x+beta) - alpha/beta + gama*mag



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

def get_bearing(lat1, long1, lat2, long2):
    dLon = (long2 - long1)
    x = cos(radians(lat2)) * sin(radians(dLon))
    y = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(dLon))
    brng = np.arctan2(x,y)
    brng = np.degrees(brng)

    return brng
def check_arm(dronelist):
    for i in dronelist:
        if ( not (i.vehicle.is_armable)):
            return 0
    return 1
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

    drn.vehicle.simple_goto(a_location,10)
    print("goto for sysid "+str(drn.get_sys_id()))
    time.sleep(10)
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
    if(do>dk):
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
    sysid = drn.get_sys_id()
    for i in range(len(dronelist)):
        if(i+1 == sysid):
            continue
        clat,clong= drn.get_pos()
        dlat,dlong = dronelist[i].get_pos()
        do = calc_dist_geo(clat,clong,dlat,dlong)
        bear = get_bearing(clat, clong, dlat, dlong)
        vr = drn.get_vel2()
        dist_vec = [clat - dlat,clong - dlong]
        # vr = reslove_vel_ned(vr[0],vr[1],drn.heading())
        vt = dronelist[i].get_vel2()
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



def follow_trajectories(sysid,drn,dronelist):

    f = open("/home/kartikey/Kartikey_swarm/drone_"+str(sysid)+".txt",'w+')
    file = open('/home/kartikey/Kartikey_swarm/traj'+str(sysid)+'.csv')
    csvreader = csv.reader(file)
    dno = len(dronelist)
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
    start_time = time.time()
    # for i in range()
    print("starting to follow")
    print("############################")
    if(flag == 0):
        flag =1
        time.sleep(1.5)
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
            print("#####*######### Updated J for sysid "+str(sysid)+" to "+str(j)+("###########"))
            j+=1
            continue
        t1 = start_time +rows[j][4] - time.time()
        if(t1<=0):

            u1 =maxu

        else:
            u1 = ((2*(dist1))/t1) - rows[j][5]
        if(u1>maxu):
            u1 = maxu
        bear1 = get_bearing(clat,clong,dlat,dlong)
        print("params for sysid "+str(sysid)+" "+str(clat)+" "+str(clong)+" "+str(dlat)+" "+str(dlong)+" bear:- "+str(bear1)+" dist:- "+str(dist1)+" u is "+str(u1))
        print("time is for sysid "+str(sysid)+ " is "+str(t1))
        nvel = u1*cos((bear1*pi)/180)
        evel = u1*sin((bear1*pi)/180)
        avoid_nvel=0
        avoid_evel=0
        kobsti = 649.152
        if(drn.get_sys_id() == 1 ):

            avoid_nvel , avoid_evel = generate_avoid_velo([2,4],dronelist,drn,maxu,kobsti)
            print("the avoid velocities for sysid "+ str(sysid)+" are ",str(avoid_nvel)," ",str(avoid_evel))
            st = str("the avoid velocities for sysid "+ str(sysid)+" are "+str(avoid_nvel)+" "+str(avoid_evel)+"\n")
            f.write(str(st))

        # if(drn.get_sys_id() == 2 ):

        avoid_nvel , avoid_evel = generate_avoid_velo([1,3,4],dronelist,drn,maxu,kobsti)
        print("the avoid velocities for sysid "+ str(sysid)+" are ",str(avoid_nvel)," ",str(avoid_evel))
        st = str("the avoid velocities for sysid "+ str(sysid)+" are "+str(avoid_nvel)+" "+str(avoid_evel)+"\n")
        f.write(str(st))


        print("velocity is "+str(nvel)+" "+str(evel))
        print("avoid velocity for sysid " +str(sysid)+" is "+ str(avoid_nvel) + " , "+ str(avoid_evel))
        velocity_mav = drn.get_vel2()
        print("mav velocity is " , str(velocity_mav))
        print("the heading is ", str(drn.heading()))
        print("velocity for sysid " +str(sysid)+" obtainded is "+str(reslove_vel_ned(velocity_mav[0],velocity_mav[1],drn.heading())))
        nvel +=avoid_nvel
        evel += avoid_evel
        drn.update_vel([nvel,evel,0])
        # st = ""
        # st = str(drn.get_pos())+" ,#333,circle"
        # print(st)

    print("the final value of j for sysid " +str(sysid)+ "  is " + str(j))
    drn.update_vel([0,0,0])
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

    print("###############")
    print("following the trajectories now")
    thread_list_trajectory = []
    for i in range(len(dronelist)):
        thread_list_trajectory.append(threading.Thread(target=follow_trajectories,args=(i+1,dronelist[i],dronelist)))

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

    # file = open('~/Kartikey_swarm/traj1.csv')
    # csvreader = csv.reader(file)
# to do how to close subprocess
# how to run different terminal processes using subprocess
# how does a thread end




# can genrate a  path wiht dynamic formation control(of uavs )
