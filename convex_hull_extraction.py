import cv2
import numpy as np
from matplotlib import pyplot as plt
# from scipy.interpolate import make_interp_spline
from scipy.spatial import ConvexHull
from shapely.geometry import  LineString
from math import *
from math import sqrt
from scipy.signal import savgol_filter
import pickle

def find_intersection(x_c,y_c,line_1,plt):

    line_2 = LineString(np.column_stack((x_c, y_c))) # line string of the above circle
    # print(black_coord)
    # print(len(black_coord))
    # print(blackAndWhiteImage)
    # print(r)
    intersection = line_1.intersection(line_2) # finding the point(s) of intersection
    # print(type(intersection))
    # print(len(intersection))
    # plt.plot(*intersection.xy, 'ro')
    # the below conditions check for if there are multiple points of intersections or not
    # and return in the desired format
    # x,y = array('d', [120.06999593331295]) array('d', [272.0])
    if intersection.geom_type == 'MultiPoint':
        plt.plot(*LineString(intersection).xy, 'o',color ="red" )
        x, y = LineString(intersection).xy

    elif intersection.geom_type == 'Point':
        plt.plot(*intersection.xy, 'o',color ="red")
        x, y = intersection.xy # for intersection of  one point
    return x,y

def create_circle(c_x,c_y,plt,theta,line1,radius): # c_x and c_y are the centre coordinaet

    # theta is array of angles to make a circle, line1 is linestring of original curve
    # Generating x and y data
    x_c = radius * np.cos(theta) + c_x
    y_c = radius * np.sin(theta) + c_y

    # Plotting
    plt.plot(x_c, y_c,color="peru") # ploting the circle
    # plt.axis('equal')
    # plt.title('Circle')
    # plt.show()
    # line_1 = LineString(np.column_stack((x, y)))
    # print(type(x[0]))
    x, y =find_intersection(x_c,y_c,line1,plt)
    return x,y

def max_dist_p(res_form,ref_x,ref_y):

    res_x,res_y = res_form[0],res_form[1]
    max_t = -10e8
    max_x =0
    for tj in range(len(res_x)):
        k = res_x[tj]
        j = res_y[tj]
        if(sqrt((ref_x-k)**2+(ref_y-j)**2) > max_t):

            max_t = sqrt((ref_x-k)**2+(ref_y-j)**2)
            max_x = k
    max_ind = res_x.index(max_x)
    center = [res_x[max_ind],res_y[max_ind]]
    print("dist is "+str(max_t))
    return abs(max_t)


originalImage = cv2.imread('/home/kartikey/Kartikey_swarm/captured.png')
grayImage = cv2.cvtColor(originalImage, cv2.COLOR_BGR2GRAY)
f = open("/home/kartikey/Kartikey_swarm/img_pix.txt",'w+')
(thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
blackAndWhiteImage  = np.array(blackAndWhiteImage,dtype=np.float32)/255.0

cv2.imshow('Black white image', blackAndWhiteImage)
# cv2.imshow('Original image',originalImage)
# cv2.imshow('Gray image', grayImage)
r = blackAndWhiteImage[10][10]
# print(blackAndWhiteImage[200,200])
black_coord = {"23","29"}
black_np = []
print(str(len(blackAndWhiteImage))+str(", ")+str(len(blackAndWhiteImage[0])))
for i in range(len(blackAndWhiteImage)):
    for j in range(len(blackAndWhiteImage[0])):
        if(blackAndWhiteImage[i][j] == 0.0):
            black_coord.add(str([i,j]))
            black_np.append([len(blackAndWhiteImage)-i,j]) # as in opencv the origin is at top,left corner
            f.write(str(blackAndWhiteImage[i][j])+" ")

    f.write("\n")

black_np = np.asarray(black_np)
black_np  = black_np[np.argsort(black_np[:,1])] # sorting the 2d array wrt x(2nd column)
x = black_np[:,1]
y = black_np[:,0]
# xy = np.hstack((x[:,np.newaxis],y[:,np.newaxis]))
#
# X_Y_Spline = make_interp_spline(x, y)
# X_ = np.linspace(x.min(), x.max(), 500)
# Y_ = X_Y_Spline(X_)
# plt.plot(X_,Y_)

# hull = ConvexHull(xy)
# plt.scatter(x,y,color="red")
# plt.plot(x[hull.vertices],y[hull.vertices])

# convex hull :-

# # RANDOM DATA
# #x = np.random.normal(0,1,100)
# #y = np.random.normal(0,1,100)
# xy = np.hstack((x[:,np.newaxis],y[:,np.newaxis]))
#
# # PERFORM CONVEX HULL
xy = np.hstack((x[:,np.newaxis],y[:,np.newaxis]))
hull = ConvexHull(xy)

# PLOT THE RESULTS
# plt.scatter(x,y)
# plt.show()
# the convex hull approch should cosider the parameter of farthest from previous point(centre) and some other point also as if
# one is considring circle than every point of intersection is  at equal distance
# and not the x value parameter

plt.figure()
# plt.plot(x,y)
# ploting the first plot
plt.plot(x[hull.vertices], y[hull.vertices])
print("the size of convex hull is " ,str([len(x[hull.vertices]), len(y[hull.vertices])] ))

x,y = x[hull.vertices], y[hull.vertices]
# black_np  = black_np[np.argsort(black_np[:,1])] # sorting the 2d array wrt x(1st column)
print(type(x))
tkr = np.array([x,y])
# tkr = tkr[np.argsort(tkr[0])] # sorting the 2d array wrt x(1st column)
tkr = tkr[:, tkr[0, :].argsort()]
# xhat = savgol_filter(x, 203, 3) # applying Savitzky–Golay filter
# yhat = savgol_filter(y, 203, 3) # window size 51, polynomial order 3
# x,y = x[hull.vertices], y[hull.vertices]


# plt.plot(x,y)
# plt.plot(xhat,yhat, color='red')
# plt.show()
# x,y = xhat,yhat
line_1 = LineString(np.column_stack((x, y)))

theta = np.linspace(0, 2 * np.pi, 100) # for creating circle

dno =15 # number of uavs
#

curve_len_pix = sqrt((tkr[0][-1]-tkr[0][0])**2 + (tkr[1][-1]-tkr[1][0])**2)

radius = curve_len_pix//dno # setting the radius

print("radius is " ,radius)
center = [x[0],y[0]] # starting from the left edge of the curve
center_i = [x[0],y[0]] # storing the first point

res_form = []

dno+=1

# getting the point of intersection vv
ti =0
max_t = -10e7
# while( len(res_form) == 0 or (max((create_circle(res_form[-1][0],res_form[-1][1],plt,theta,line_1,radius))[0])) > res_form[-1][0]  ):

while(( len(res_form) == 0 or (max_dist_p((create_circle(res_form[-1][0],res_form[-1][1],plt,theta,line_1,radius)),center_i[0],center_i[1])) > 2*radius ) ) : #and radius <85

    center = [x[0],y[0]]
    print(ti)
    ti+=1
    res_form = []
    print("radius is "+str(radius))

    for i in range(dno):

        max_t = -10e9
        max_x =0

        res_form.append([center[0],center[1]])

        if(i<2):
            res_x,res_y = create_circle(center[0],center[1],plt,theta,line_1,radius) # intersection points

            max_ind = res_x.index(max(res_x)) # dikkat
            center = [res_x[max_ind],res_y[max_ind]]
        else:
            max_t = -10e9
            res_x,res_y = create_circle(center[0],center[1],plt,theta,line_1,radius) # intersection points
            ref_p =[(res_form[-1][0]+res_form[-2][0])/2 ,(res_form[-1][1]+res_form[-2][1])/2]
            max_x = res_x[0]
            k =0
            j =0
            # print("res_x is ")
            # print(res_x)
            # print("res_y is ")
            #
            # print(res_y)
            for phi in range(len(res_x)):
                jk = res_x[phi]
                t = res_y[phi]

                if(sqrt(((ref_p[0]-jk)**2)+((ref_p[1]-t)**2)) > max_t):
                    # print("updating")
                    max_t = sqrt(((ref_p[0]-jk)**2)+((ref_p[1]-t)**2) )
                    # max_t = sqrt((ref_p[0]-k)**2+(ref_p[1]-j)**2)
                    max_x = jk
            # print("max_x is "+str(max_x))
            # print(res_x)
            # if(max_x in res_x):
            #     print("found")
            # else:
            #     print("not found")
            max_ind = res_x.index(max_x)
            center = [res_x[max_ind],res_y[max_ind]]

    radius +=5
# c_x,c_y,plt,theta,line_1,radius
print("last element is "+str(max((create_circle(res_form[-1][0],res_form[-1][1],plt,theta,line_1,radius))[0])) )



print(res_form)
print(len(res_form))
plt.gca().set_aspect('equal',adjustable='box')
res_form = np.asarray(res_form)
# print(res_form[:,0])
t1 = res_form[0][0]
t2 = res_form[0][1]
# reducing the scale and shhifting origin
xr = (radius-5)/10
for i in range(len(res_form)):
    res_form[i][0] -= t1
    res_form[i][1] -= t2
    res_form[i][0] /=xr
    res_form[i][1] /=xr

print(type(res_form))
print("distances")
for i in range(len(res_form)-1):

    dis  = sqrt((res_form[i+1][0]  - res_form[i][0])**2 +(res_form[i+1][1]  - res_form[i][1])**2)
    print(dis)



plt.figure()
# res_form.pop()
# poping the last element
res_form=res_form[:-1]
print("the lenght of res_form is "+str(len(res_form)))
plt.scatter(res_form[:,0],res_form[:,1],marker = "^")
res_form = res_form.tolist()
print("printing res_form")
print(type(res_form))

print(res_form)
with open('/home/kartikey/Kartikey_swarm/cart_coord_cursor.pkl', 'wb') as f:
    pickle.dump(res_form, f)

f.close()
plt.gca().set_aspect('equal',adjustable='box')


# print(res_x,res_y)
plt.show()
cv2.waitKey(0)
cv2.destroyAllWindows()
