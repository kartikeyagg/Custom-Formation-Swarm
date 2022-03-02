import geopy
import geopy.distance

# Define starting point.
start = geopy.Point(-35.36325519,149.16509403)

# Define a general distance object, initialized with a distance of 1 km.
# d = geopy.distance.distance(kilometers=0.01)
#
# # Use the `destination` method with a bearing of 0 degrees (which is north)
# # in order to go from point `start` 1 km to north.
# final = d.destination(point=start, bearing=90).format_decimal()
# print("result " +str(final))
# from geopy import Point
# from geopy.distance import distance, VincentyDistance
#
# # given: lat1, lon1, bearing,
# lat1,lon1  = -35.36325,149.16509
# dist  = 0.01
# bearing  = 90
# lat2, lon2 = VincentyDistance(Kilometers=dist).destination(Point(lat1, lon1), bearing)
# print(lat2,lon2)
for i in range(50):
    d = geopy.distance.distance(kilometers=(0.01*(i+1)))

    # Use the `destination` method with a bearing of 0 degrees (which is north)
    # in order to go from point `start` 1 km to north.
    final = d.destination(point=start, bearing=90).format_decimal()
    # print(type(final))
    print("T"+str(i)+"="+str(final))
