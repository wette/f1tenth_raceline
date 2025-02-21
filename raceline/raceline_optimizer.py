import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__)))

#uses https://github.com/AtsushiSakai/pycubicspline
import pycubicspline.pycubicspline as pyspline
from trajectory import Trajectory, VehicleDescription
from map import Map

import yaml
import math
from matplotlib import pyplot 
import random

class RacelineOptimizer:
    def __init__(self, configfile: str):
        self.__config = None
        
        self.parse_config(configfile)
        #reconstruct filepath:
        path = "/".join(configfile.split("/")[0:-1])
        self.__map = Map(path + "/" + self.__config["image"], self.__config["origin"], self.__config["resolution"])

    def get_map(self) -> Map:
        return self.__map

    def get_config(self) -> dict:
        return self.__config

    def debug_draw_map(self):
        pyplot.imshow(self.__map.get_pixel_map())
        pyplot.show()

    def debug_draw_trajectory(self, trajectory : Trajectory, filename: str = None, title: str = None):
        pyplot.imshow(self.__map.get_pixel_map())

        lx,ly, _, curvature, _ = pyspline.calc_2d_spline_interpolation(trajectory.x + trajectory.x[1:2], trajectory.y + trajectory.y[1:2], num=300)

        rl = Trajectory(lx, ly, trajectory.get_vehicle_description(), trajectory.resolution, curvature=curvature)
        rl.do_forwards_pass = True
        rl.compute_velocity_profile()
        #print(f"Raceline with 300 points time: {rl.get_laptime()}")

        pyplot.scatter(rl.x,rl.y, c=rl.velocity_profile, linewidth=1, cmap=pyplot.cm.coolwarm)
        pyplot.colorbar()

        if title is not None:
            pyplot.title(title)
        
        try:
            if filename is not None:
                pyplot.savefig(filename, dpi=200)
                pyplot.clf()
            else:
                pyplot.show()
        except:
            print(f"could not save image to file {filename}")

        

    def get_manual_initial_centerline(self, num_control_points = 200) -> tuple[int, int]:
        """
            Let the user define the initial centerline for optimization by clicking with a mouse.
        """
        print("#####################")
        print("#### click in the image to give manual controlpoints to a spline")
        print("#####################")
        fig = pyplot.figure()
        ax = fig.add_subplot()
        ax.imshow(self.__map.get_pixel_map())
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
        for p in points[:]:
            if self.__map[p[1]][p[0]] > 0.0:
                points.remove(p)

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        #start and end with the same point.
        xs.append(xs[0])
        ys.append(ys[0])

        print(xs)
        print(ys)

        #interpolate:
        x, y, yaw, k, travel = pyspline.calc_2d_spline_interpolation(xs, ys, num=num_control_points)
        
        print(f"Initial Lap Length [m]: {max(travel) * self.__config['resolution']}")

        return x, y

    def parse_config(self, filename: str):
        f = open(filename, 'r')
        self.__config = yaml.safe_load(f)
        f.close()

    def optimize_raceline(self, initial_trajectory: Trajectory, turning_radius_m: float, 
                          num_epochs=250, num_keep=20, num_population=200, 
                          max_change_in_pixels=3, num_changes_per_mutation=1,
                          filename="my_map_raceline.csv", num_points_file=300, 
                          num_ctrl_points=40) -> Trajectory:
        """use a genetic algorithm to find a raceline"""

        def remove_all_but_top(population: list, num_keep: int):
            population.sort(key=lambda x : -1 * x.get_laptime()) #-1 to sort descending!
            return population[len(population)-num_keep : len(population)]

        #initialize population with random racelines deduced from the initial trajectory.
        population = []
        for i in range(num_population):
            t = initial_trajectory.copy()
            t.random_changes(max_change_in_pixels, num_changes_per_mutation, self.__map, num_ctrl_points=num_ctrl_points)
            population.append(t)
        population.append(initial_trajectory.copy()) #keep in the original one w/o modifications

        #remove all but the top racelines
        population = remove_all_but_top(population, num_keep)
        
        #go through epochs
        for e in range(0, num_epochs):

            #create new offspring
            new_childs = []
            offspring_count = int(math.ceil(num_population/num_keep))
            for rl in population:
                for i in range(offspring_count):
                    new_childs.append(rl.copy())

            #mutate offspring
            for rl in new_childs:
                rl.random_changes(max_change_in_pixels, num_changes_per_mutation, self.__map, num_ctrl_points=num_ctrl_points)
            
            new_childs += population[:] #copy over old trajectories to new childs for random_combination
            
            #combine a random pair of trajectories
            combined_childs = []
            """random.shuffle(new_childs)
            for i in range(0, int(len(new_childs)/2)):
                i = random.randint(0, len(new_childs)-1)
                j = random.randint(0, len(new_childs)-1)
                combined = new_childs[i].copy()
                combined.random_combination(new_childs[j], num_ctrl_points=num_ctrl_points)
                combined_childs.append( combined )
            """

            #add the newly mutated children to population:
            for l in new_childs:
                population.append(l)
            for l in combined_childs:
                population.append(l)

            #make sure population are NOT driving through non-free space!
            #and that turning radius is feasable for the vehicle
            max_curvature = 1.0 / (turning_radius_m/population[0].resolution)
            vehicle_width_in_map_pixels = math.ceil(population[0].vehicle_width_m / self.__config['resolution'])
            for l in population[:]:
                lx,ly, _, curvature, _ = pyspline.calc_2d_spline_interpolation(l.x, l.y, num=500)

                #check curvature:
                curvature_ok = True
                for c in curvature:
                    if abs(c) > max_curvature:
                        curvature_ok = False
                        break
                
                if not curvature_ok:
                    population.remove(l)
                    continue

                #for each point of the trajectory:
                for i in range(len(lx)):
                    #check if a square around each point of the trajectory is all in free space
                    #TODO: This should actually be a circle!
                    removeTrajectory = False
                    if self.__map[int(ly[i])][int(lx[i])] > 0.0:
                        #print(f"Point {i} not in free space: {int(lx[i])},{int(ly[i])}")
                        removeTrajectory = True
                        break
                    for dx in range(-math.floor(vehicle_width_in_map_pixels/2.0), math.ceil(vehicle_width_in_map_pixels/2.0), 1):
                        for dy in range(-math.floor(vehicle_width_in_map_pixels/2.0), math.ceil(vehicle_width_in_map_pixels/2.0), 1):
                            if self.__map[int(ly[i]+dy)][int(lx[i]+dx)] > 0.0:
                                #print(f"Point {i} not in free space: {int(ly[i]+dy)},{int(lx[i]+dx)}")
                                removeTrajectory = True
                                break
                        if removeTrajectory:
                            break
                    if removeTrajectory:
                            break

                if removeTrajectory:
                    population.remove(l)

            if len(population) == 0:
                raise Exception("Not possible to find a raceline. Check parameters and initial raceline.")
            
            print(f"Valid Population size: {len(population)}")

            #remove all but the top racelines
            population = remove_all_but_top(population, num_keep)

            #print length of best raceline.
            laptime_s = population[-1].get_laptime() * self.get_map().get_resolution()
            print(f"raceline length/laptime in epoch {e}: {population[-1].get_length() * self.get_map().get_resolution()} / {laptime_s}")
            self.debug_draw_trajectory(population[-1], f"racelines/racelineEpoch{e}.png", title=f"Epoch {e} - Laptime {laptime_s}s")
            population[-1].safe_trajectory_to_file(self.get_map(), filename, num_points=num_points_file)
            
        
        population[-1].safe_trajectory_to_file(self.get_map(), filename, num_points=num_points_file)
        return population[-1]
    

def main():
    haftreibung                 = 6.0   #kg force to move standing vehicle in lateral direction
    vehicle_mass_kg             = 3.0
    vehicle_width_m             = 0.6#0.3      #half width is minimum distance to any wall at any time
    vehicle_length_m            = 0.5
    vehicle_acceleration_mss    = 7.0     #vehicle acceleration in meters/sec/sec
    vehicle_deceleration_mss    = 3.0     #vehicle deceleration in meters/sec/sec
    turning_radius_m            = 1.2/2.0 #1.0/2.0      #turning radius of the vehicle in meters
    min_steering_angle_deg      = -25
    max_steering_angle_deg      = 25

    desired_points_per_meter    = 1.0      #how many control points to use during optimization per spline (you want as few as possible!)
    max_change_per_point_meters = 0.5      #how much change to a controlpoint per iteration in meters (should be pretty small; few cm)

    num_epochs                  = 25       #number of optimization epochs
    num_keep                    = 50      #number of trajectories to keep after each epoch
    num_population              = 1000     #population size during epoch
    num_changes_per_mutation    = 2        #number of controlpoint changes during a mutation


    opt = RacelineOptimizer("/home/wette/wette_racecar_ws/MindenCitySpeedway.yaml")
    
    #for development: fixed start trajectory - in production this should come from waypoints sampled from follow the gap algorithm.
    #x,y = opt.get_manual_initial_centerline()

    #minden city speedway initial trajectory
    x = [306, 328, 347, 356, 361, 365, 367, 367, 362, 353, 342, 328, 308, 296, 286, 279, 272, 263, 252, 234, 221, 218, 217, 221, 231, 240, 248, 249, 242, 230, 214, 194, 183, 167, 154, 143, 131, 122, 114, 114, 118, 136, 149, 166, 176, 179, 173, 162, 152, 150, 152, 167, 193, 223, 261, 285]
    y = [46, 44, 46, 56, 72, 90, 111, 129, 145, 156, 166, 168, 167, 156, 142, 129, 118, 107, 100, 97, 101, 114, 125, 133, 140, 144, 153, 169, 178, 180, 178, 175, 172, 172, 177, 181, 183, 182, 171, 159, 151, 146, 146, 142, 133, 120, 108, 101, 92, 81, 68, 61, 59, 55, 52, 49]
    #x = [177, 324, 349, 361, 366, 368, 362, 344, 324, 301, 289, 277, 265, 252, 231, 218, 220, 235, 248, 244, 221, 200, 181, 166, 151, 135, 121, 113, 116, 135, 156, 172, 177, 171, 157, 152, 162]
    #y = [59, 46, 51, 69, 94, 124, 147, 163, 168, 163, 145, 123, 106, 98, 98, 111, 131, 142, 155, 173, 177, 175, 172, 173, 178, 183, 184, 173, 156, 149, 145, 138, 123, 108, 96, 75, 63]

    """#max_change_per_point_meters must be smaller than half desired_points_per_meter
    if desired_points_per_meter/2 < (max_change_per_point_meters+0.1):
        print("Error: max_change_per_point_meters is too large!")
        return
    """
    #add the first point of the spine as last, too -> circle
    x.append(x[0])
    y.append(y[0])
    x,y, _, _, path_len = pyspline.calc_2d_spline_interpolation(x, y, num=100)
    
    #compute number of control points:

    track_in_meters = path_len[-1] * opt.get_config()["resolution"]
    num_ctrl_points = math.ceil(track_in_meters * desired_points_per_meter)

    #resample spline with desired number of control points:
    x,y, _, _, path_len = pyspline.calc_2d_spline_interpolation(x, y, num=num_ctrl_points)

    vd = VehicleDescription(haftreibung, vehicle_width_m,  vehicle_mass_kg, vehicle_acceleration_mss, vehicle_deceleration_mss, vehicle_length_m, min_steering_angle_deg, max_steering_angle_deg)
    original = Trajectory(x, y, vd, opt.get_config()["resolution"])

    #use genetic algorithm to optimize x,y
    raceline = opt.optimize_raceline(initial_trajectory=original, turning_radius_m=turning_radius_m, num_epochs=num_epochs, num_keep=num_keep, num_population=num_population, 
                                    num_changes_per_mutation=num_changes_per_mutation, 
                                    max_change_in_pixels=max_change_per_point_meters/opt.get_config()["resolution"],
                                    filename="my_map_raceline.csv", num_points_file=300,
                                    num_ctrl_points=num_ctrl_points)

    raceline.laptime = None
    raceline.do_forwards_pass = True
    print(f"Raceline time: {raceline.get_laptime()}")
    opt.debug_draw_trajectory(raceline, title="")


if __name__ == "__main__":
    main()