
# Python3 program to find the CIRCUMCENTER of a
# triangle

# This pair is used to store the X and Y
# coordinate of a point respectively
# define pair<double, double>

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
# Driver code.
if __name__ == '__main__':
    P = [23.792347, 78.793324]
    Q = [23.792436, 78.778187]
    R = [23.814476, 78.786380]
    res = findCircumCenter(P, Q, R)
    print(str(P[0])+","+str(P[1])+",red,circle")
    print(str(Q[0])+","+str(Q[1])+",red,circle")
    print(str(R[0])+","+str(R[1])+",red,circle")
    print(str(res[0])+","+str(res[1])+",green,circle")
