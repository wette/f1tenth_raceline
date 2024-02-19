from trajectory import Trajectory
import pycubicspline.pycubicspline as pyspline

class Map:
    def __init__(self, image_file: str, origin: list, resolution: float):
        self.__map = self.parse_image(image_file)

        self.__origin_x = int(origin[0] / resolution * -1.0)
        self.__origin_y = int(origin[1] / resolution * -1.0)

        self.__origin = origin.copy()
        self.__resolution = resolution

        self.binarize_image()
        self.region_growing()

    def safe_trajectory_to_file(self, trajectory: Trajectory, file: str, num_points: int):
        """ write computed trajectory to file """

        f = open(file, "w")
        f.write("x\ty\tvelocity[m]\n")

        x,y, _, _, _ = pyspline.calc_2d_spline_interpolation(trajectory.x, trajectory.y, num=num_points)

        trajectory = Trajectory(x, y, trajectory.haftreibung, trajectory.vehicle_width_m, trajectory.vehicle_acceleration_mss, trajectory.vehicle_deceleration_mss, trajectory.resolution)
        trajectory.compute_velocity_profile()

        for i in range(len(trajectory.x)):
            x_px = trajectory.x[i]
            y_px = trajectory.y[i]

            #transform pixel to coordinates
            #TODO: Check if x and y should be swapped around!
            x = x_px * self.__resolution + self.__origin[0]
            y = y_px * self.__resolution + self.__origin[1]

            f.write(f"{x}\t{y}\t{trajectory.velocity_profile[i]}\n")

        f.close()
        

    #override []-operator
    def __getitem__(self, key):
        return self.__map[key]
    
    def get_pixel_map(self):
        return self.__map

    def parse_image(self, image_file: str):
        """Return a raster of integers from a PGM as a list of lists.
           from https://stackoverflow.com/questions/35723865/read-a-pgm-file-in-python""" 
        pgmf = open(image_file, "r", encoding="latin-1")
        firstline = pgmf.readline()
        assert firstline == 'P5\n'
        (width, height) = [int(i) for i in pgmf.readline().split()]
        depth = int(pgmf.readline())
        assert depth <= 255

        raster = []
        for y in range(height):
            row = []
            for x in range(width):
                row.append(ord(pgmf.read(1)))
            raster.append(row)
        return raster
    
    def binarize_image(self):
        """use the thresholds to create a 0/1 image showing freespace as 0"""
        threshold_free = 220
        for x in range(0, len(self.__map)):
            for y in range(0, len(self.__map[0])):
                if self.__map[x][y] >= threshold_free:
                    self.__map[x][y] = 0
                else:
                    self.__map[x][y] = 1

    def region_growing(self):
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
            for i in range(len(n)-1,0, -1):
                if n[i][0] < 0 or n[i][0] > max_x or n[i][1] < 0 or n[i][1] > max_y:
                    n.remove( n[i] )

            return n

        #BFS
        queue = [ (self.__origin_x, self.__origin_y) ]
        self.__map[self.__origin_x][self.__origin_y] = visited_marker
        while len(queue) > 0:
            cur = queue.pop(0)
            for x,y in neighbors(cur[0], cur[1], len(self.__map)-1, len(self.__map[0])-1):
                if self.__map[x][y] == 0:
                    self.__map[x][y] = visited_marker
                    queue.append( (x,y) )

        #flip back visited_marker to 0
        for x in range(len(self.__map)):
            for y in range(len(self.__map[0])):
                if self.__map[x][y] == visited_marker:
                    self.__map[x][y] = 0
                else:
                    self.__map[x][y] = 1