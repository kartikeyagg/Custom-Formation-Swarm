import numpy as np
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




lat1 = [28.718771,28.718790,28.718643]
long1 = [77.174714,77.174977,77.174858]

# 28.718771, 77.174714
# 28.718790, 77.174977
# 28.718643, 77.174858

res = GetCenterFromDegrees(lat1,long1)
print(res)
# res  = (28.71873466670919, 77.17484966666235)
