import numpy as np
from math import *
import matplotlib.pyplot as plt
alpha = 100
Beta = 1.25
gama = 2.1
delt = 15
rho = 0.50
d0 =0
d1 = 0
amax = 3
beta = Beta

def u_rep(x,vr,vt):

    vr = np.asarray(vr)
    vt = np.asarray(vt)
    diff = vr-vt
    print("diff is " + str(diff))
    mag =( np.linalg.norm(diff))**2
    print("magnitude is "+str(mag))
    U = (alpha/beta**2)*x + alpha/(x+beta) - alpha/beta + gama*mag
    return U
def u_att(x,vr,vt):

    vr = np.asarray(vr[0:2])
    vt = np.asarray(vt)
    diff = vr-vt
    mag =( np.linalg.norm(diff))**2
    U = (alpha/beta**2)*x + alpha/(x+beta) - alpha/beta + gama*mag
    v = (U)**0.45
    return v
def bugs(do,kobsti,maxu,vr,vt):

    vr = np.asarray(vr)
    vt = np.asarray(vt)
    diff = vr-vt
    print("diff is " + str(diff))
    mag =( np.linalg.norm(diff))**2
    print("mag is "+str(mag))

    dk = 8 # the distance for which the uav starts avoiding
    U = 0
    if(do>dk):
        pass
    else:
        U = 0.5*kobsti*((1/do - 1/dk)**1.7) + gama*mag

    print("U is ",str(U))

    return sqrt(U)



beta = 0.1
beta = 3.6
alpha =50
x = 7
vr = [6.3,8]
vt = [6,6.1]
U= u_rep(x,vr,vt)
alph_t  =[]
U_t = []
for i in range (100):
    U_t.append(u_att(x,vr,vt))
    alph_t.append(x)

    x +=0.5

print(alpha,beta)
print(alph_t)
plt.plot(alph_t,U_t,color="green")
plt.title("Alpha change dis =10m")
plt.xlabel("alpha")
plt.ylabel("v m/s")
plt.show()


# t = 10
#
# def tt():
#     t = 0
#     t+=1
#     print(t)
#
# tt()




#
# print("result "+str(U))
#
# vel = []
# tk = []
# maxu =12
# kobsti = 649.152
# vr = [11.3,2]
# vt = [12,2.1]
# for i in range(70):
#     tk.append([(i+1)*0.1,kobsti,vr,vt])
# for i in tk:
#     vel.append(bugs(i[0],i[1],maxu,i[2],i[3]))
#
# # print(vel)
# vel_n = np.asarray(vel)
# dist_a  =[row[0] for row in tk]
# dist_a = np.asarray(dist_a)
# print(vel_n)
# print(dist_a)
# plt.plot(dist_a,vel_n,color="orange")
# vr = [3,3]
# vt = [2,1]
# vr = np.asarray(vr)
# vt = np.asarray(vt)
# diff = vt-vr
# sysid =1
# print("diff is "+str(diff))
# print("the mag is " +str(np.linalg.norm(diff)))
# f = open("/home/kartikey/Kartikey_swarm/drone_"+str(sysid)+".txt",'w+')
# f.write("yoyo")
# f.close()
# plt.show()
