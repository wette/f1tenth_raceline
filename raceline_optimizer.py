#uses https://github.com/AtsushiSakai/pycubicspline
import pycubicspline.pycubicspline as pyspline


import numpy as np
import yaml
import math
from matplotlib import pyplot 

#x = np.linspace(0, 10, num=11)
#y = np.cos(-x**2 / 9.)
#spl = CubicSpline(x, y)

#karte holen.
# scp "192.168.64.5:/home/wette/cartographer_ws/my_map*" .

class RacelineOptimizer:
    def __init__(self, configfile: str):
        self.__config = None
        
        self.parse_config(configfile)
        self.__map = self.parse_image()

        self.__origin_x = int(self.__config["origin"][0] / self.__config["resolution"] * -1.0)
        self.__origin_y = int(self.__config["origin"][1] / self.__config["resolution"] * -1.0)

        self.binarize_image(self.__map)
        self.region_growing(self.__map)

    def debug_draw_map(self):
        pyplot.imshow(self.__map)
        pyplot.show()

    def get_manual_initial_centerline(self, num_control_points = 200) -> (int, int):
        print("#####################")
        print("#### click in the image to give manual controlpoints to a spline")
        print("#####################")
        fig = pyplot.figure()
        ax = fig.add_subplot()
        ax.imshow(self.__map)
        points = []

        def onclick(event):
            print('add point x=%f, y=%f' % (event.xdata, event.ydata))
            p = [int(event.xdata), int(event.ydata)]
            ax.plot([event.xdata],[event.ydata],"bo")
            fig.canvas.draw()
            points.append(p)

        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        pyplot.show()

        #remove all points which are not within free space
        for p in points:
            if self.__map[p[0]][p[1]] != 0:
                points.remove(p)

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        #start and end with the same point.
        xs.append(xs[0])
        ys.append(ys[0])

        #interpolate:
        x, y, yaw, k, travel = pyspline.calc_2d_spline_interpolation(xs, ys, num=num_control_points)
        
        print(f"Initial Lap Length [m]: {max(travel) * self.__config['resolution']}")

        pyplot.imshow(self.__map)
        pyplot.plot(x,y)
        pyplot.show()


        return x, y


    def region_growing(self, image: list):
        """use region growing to remove unreachable parts of the map"""
        visited_marker = 256

        #return neighbors in 4-neighborhood (up, down, left, right)
        def neighbors(x, y, max_x, max_y):
            n = []
            n.append( (x-1, y) )
            n.append( (x+1, y) )
            n.append( (x, y+1) )
            n.append( (x, y-1) )

            #filter out corners!
            for i in range(0, len(n)):
                if n[i][0] < 0 or n[i][0] > max_x or n[i][1] < 0 or n[i][1] > max_y:
                    n.remove( n[i] )

            return n

        #BFS
        queue = [ (self.__origin_x, self.__origin_y) ]
        image[self.__origin_x][self.__origin_y] = visited_marker
        while len(queue) > 0:
            cur = queue.pop(0)
            for x,y in neighbors(cur[0], cur[1], len(image)-1, len(image[0])-1):
                if image[x][y] == 0:
                    image[x][y] = visited_marker
                    queue.append( (x,y) )

        #flip back visited_marker to 0
        for x in range(len(image)):
            for y in range(len(image[0])):
                if image[x][y] == visited_marker:
                    image[x][y] = 0
                else:
                    image[x][y] = 1

    def binarize_image(self, image: list):
        """use the thresholds to create a 0/1 image showing freespace as 0"""
        threshold_free = 220
        for x in range(0, len(image)):
            for y in range(0, len(image[0])):
                if image[x][y] >= threshold_free:
                    image[x][y] = 0
                else:
                    image[x][y] = 1

    def parse_config(self, filename: str):
        f = open(filename, 'r')
        self.__config = yaml.safe_load(f)
        f.close()

    def parse_image(self):
        """Return a raster of integers from a PGM as a list of lists.
           from https://stackoverflow.com/questions/35723865/read-a-pgm-file-in-python""" 
        pgmf = open(self.__config["image"], "r", encoding="latin-1")
        firstline = pgmf.readline()
        assert firstline == 'P5\n'
        (width, height) = [int(i) for i in pgmf.readline().split()]
        depth = int(pgmf.readline())
        assert depth <= 255

        raster = []
        for y in range(height):
            row = []
            for y in range(width):
                row.append(ord(pgmf.read(1)))
            raster.append(row)
        return raster


def main():
    opt = RacelineOptimizer("my_map.yaml")
    x,y = opt.get_manual_initial_centerline()

    #TODO: use genetic algorithm to optimize x,y

if __name__ == "__main__":
    main()