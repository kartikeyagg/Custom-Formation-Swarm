# this programe aims to generate probability map of given (or randomly generated ) targets
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

EXTEND_AREA = 10.0  # [m] grid map extention length

show_animation = True


def generate_gaussian_grid_map(ox, oy, xyreso, std):

    minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(ox, oy, xyreso) # this generates the end points and starting points of the map to be
    # observed
    # xw and yw is the width of the x and y line(no of pixels(or basic unit area))

    gmap = [[0.0 for i in range(yw)] for i in range(xw)] # intializing every cell as 0 (white)
    #iterating through every cell
    for ix in range(xw):
        for iy in range(yw):

            x = ix * xyreso + minx # this gives coordinates of cell in cartesian plane
            y = iy * xyreso + miny

            # Getting the distance from nearest cross(target)
            mindis = float("inf")
            for (iox, ioy) in zip(ox, oy):
                d = math.hypot(iox - x, ioy - y)
                if mindis >= d:
                    mindis = d

            pdf = (1.0 - norm.cdf(mindis, 0.0, std)) # cds :- cumulative distribution function
            # pdf :- probability density function
            gmap[ix][iy] = pdf

    return gmap, minx, maxx, miny, maxy


def calc_grid_map_config(ox, oy, xyreso):
    minx = round(min(ox) - EXTEND_AREA / 2.0) # this function defines the edge of the graph to be observed
    miny = round(min(oy) - EXTEND_AREA / 2.0)
    maxx = round(max(ox) + EXTEND_AREA / 2.0)
    maxy = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((maxx - minx) / xyreso)) # no of cell in x direction
    yw = int(round((maxy - miny) / xyreso))

    return minx, miny, maxx, maxy, xw, yw


def draw_probmap(data, minx, maxx, miny, maxy, xyreso):
    # slice syntax : -slice(start, end, step)
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    plt.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    plt.axis("equal")


def main():
    print(__file__ + " starting !!")
    f= open("/home/kartikey/Kartikey_swarm/prob_map.txt","w+") # for loffing

    xyreso = 0.5# is working for 0.2  # xy grid resolution
    # the above param should be less than 1
    STD = 5.0  # standard diviation for gaussian distribution
    n_target = 4

    for i in range(5):
        ox = (np.random.rand(n_target) - xyreso) * 10.0 # generating n_target random coordinates in the given area
        oy = (np.random.rand(n_target) - xyreso) * 10.0
        # getting the map
        gmap, minx, maxx, miny, maxy = generate_gaussian_grid_map(
            ox, oy, xyreso, STD)
        print("gmap size is "+str(len(gmap))+" "+str(len(gmap[0])))

        if show_animation:  # pragma: no cover
            plt.cla() # clears the plot for new one
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            draw_probmap(gmap, minx, maxx, miny, maxy, xyreso)# it draws the heat(probability map)
            # print(gmap)
            f.write(str(gmap)+str("\n\n"))
            plt.plot(ox, oy, "xr") # marking the points of interests as red X
            plt.plot(0.0, 0.0, "ob") # marking the origin wiht blue circle dot
            plt.pause(2.0)


if __name__ == '__main__':
    # print((np.random.rand(4) - 0.5))
    main()
