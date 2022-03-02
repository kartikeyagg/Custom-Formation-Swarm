import math
import numpy as np
def trialter(LatA,LonA,DistA,LatB,LonB,DistB,LatC,LonC,DistC):
    #assuming elevation = 0
    earthR = 6371
    LatA = 37.418436
    LonA = -121.963477
    DistA = 0.265710701754
    LatB = 37.417243
    LonB = -121.961889
    DistB = 0.234592423446
    LatC = 37.418692
    LonC = -121.960194
    DistC = 0.0548954278262

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
