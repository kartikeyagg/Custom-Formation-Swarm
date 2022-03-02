#
# k =0
# for i in range(20):
#     if(i>k):
#         k = i
# print(i)
# print(k)
def cdnt_to_bearing(ls):
    res = []
    for i in range(1,len(ls)):
        bear = atan2(ls[i][0],ls[i][1]) # dont forget that this is the angle with y axis and
        # x so x,y
        bear  =degrees(bear)
        # bear =90-bear
        disti = sqrt(ls[i][0]**2 + ls[i][1]**2)
        if(ls[i][0]<0):
            bear*=-1

        res.append([bear,disti])

    return res
