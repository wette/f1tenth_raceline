import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__)))
import pycubicspline.pycubicspline as pyspline

import yaml
import math
import time

from map import Map
from trajectory import Trajectory, VehicleDescription
from matplotlib import pyplot



class MPCController():
    def __init__(self, mapconfigfile: str, raceline: str, vehicle_description: VehicleDescription, 
                 lookahead_m: int = 10, points_per_meter: int = 15, candidate_count: int = 20):
        
        self.FigCounter = 0
        self.__config = None
        self.parse_config(mapconfigfile)
        #reconstruct filepath:
        path = "/".join(mapconfigfile.split("/")[0:-1])

        #map which is static only
        self.__static_map = Map(path + "/" + self.__config["image"], self.__config["origin"], self.__config["resolution"])
        
        #map which is joined static map and dynamic laser observations
        self.__map = self.__static_map.copy() #no laser observations, yet.



        self.__lookahead_m = lookahead_m
        self.__points_per_meter = points_per_meter
        self.__candidate_count = candidate_count


        self.__current_trajectory_fine = None #trajectory we are folowing
        self.__current_trajectory_coarse = None #trajectory we are folowing
        self.__last_added_raceline_point = None

        #raceline
        print(f"loading raceline from {raceline}...", flush=True)
        self.raceline = Trajectory(x=[0, 1, 2], y=[0, 1, 2], vehicle_description=vehicle_description, resolution=0, leave_in_cycle=True)
        self.raceline.load_trajectory_from_file(raceline) #meters - not pixels!

        #transform map from meters (and some origin) into pixels with origin (0,0)
        print(f"transforming raceline with {len(self.raceline.x)} points...", flush=True)
        max_x, max_y = self.__map.get_map_size_pixels()
        resolution = self.__map.get_resolution()
        origin = self.__map.get_origin()
        for i in range(len(self.raceline.x)):
            x_px = self.raceline.x[i] / resolution
            y_px = self.raceline.y[i] / resolution

            #transform pixel to coordinates - y axis needs to be swapped around!
            self.raceline.x[i] = x_px  - origin[0] / resolution
            self.raceline.y[i] = (max_x - y_px)  + origin[1] / resolution    #TODO: Shouldn't this be max_y? Somewhere down the line x and y seem to be swapped by mistake!

        #extract controlpoints from raceline (every 0.25 meters):
        print(f"resampling raceline to {int(self.raceline.get_length() * self.__config['resolution']*1)} points...", flush=True)
        self.controlpoints_raceline_x,self.controlpoints_raceline_y, _, _, _ = pyspline.calc_2d_spline_interpolation(self.raceline.x, 
                                                                                                                     self.raceline.y, 
                                                                                                                     num=int(self.raceline.get_length() * self.__config["resolution"]*1))
                                                                                                                     

        self.__vehicle_description = vehicle_description
        

        #internal state
        self.index_on_raceline = -1
        self.vehicle_x = None
        self.vehicle_y = None
        self.vehicle_yaw = None

        print(f"ready.", flush=True)

    def get_fine_trajectory_meters(self, numEntries=10):
        if self.__current_trajectory_fine is None:
            return None
        
        raceline = self.__current_trajectory_fine.copy()

        raceline.x = raceline.x[0:numEntries]
        raceline.y = raceline.y[0:numEntries]
        if raceline.velocity_profile is not None:
            raceline.velocity_profile = raceline.velocity_profile[0:numEntries]


        #transform from pixel space to meters
        origin = self.__map.get_origin()
        max_y, max_x = self.__map.get_map_size_pixels()
        for i in range(len(raceline.x)):
            x_px = raceline.x[i]
            y_px = raceline.y[i]

            #transform pixel to coordinates - y axis needs to be swapped around!
            raceline.x[i] = x_px * self.__map.get_resolution() + origin[0]
            raceline.y[i] = (max_y - y_px) * self.__map.get_resolution() + origin[1]



        return raceline
    
    def get_fine_trajectory_pixels(self, numEntries=10):
        return self.__current_trajectory_fine
    
        if self.__current_trajectory_fine is None:
            return None
        
        raceline = self.__current_trajectory_fine.copy()

        raceline.x = raceline.x[0:numEntries]
        raceline.y = raceline.y[0:numEntries]
        if raceline.velocity_profile is not None:
            raceline.velocity_profile = raceline.velocity_profile[0:numEntries]


        return raceline
    


    def callback_new_laser(self, msg, x_vehicle_map_m, y_vehicle_map_m, yaw_vehicle_map):
        #remove offset from vehicle pos
        origin = self.__map.get_origin()
        x_vehicle_map_m, y_vehicle_map_m = x_vehicle_map_m - origin[0], y_vehicle_map_m - origin[1]

        #convert from meters to pixels
        x_vehicle_map, y_vehicle_map = x_vehicle_map_m /self.__config["resolution"], y_vehicle_map_m / self.__config["resolution"]

        #swap y-axis
        max_y = self.__map.get_map_size_pixels()[0]
        y_vehicle_map = (max_y - y_vehicle_map)
        
        start = time.time()
        #transform laser message into pixel map:
        self.__map = self.__static_map.copy()

        resolution = self.__map.get_resolution()

        pxmap = self.__map.get_pixel_map()

        maxy = len(pxmap)
        maxx = len(pxmap[0])


        #print("-------")

        for i in range(0, len(msg.ranges), 2): #only use every second ray
            idx = i #looking down the x axis, incrementing to left
            angle_rad = yaw_vehicle_map - (i * msg.angle_increment + msg.angle_min) #angle in map coordinate system
            distance_m = msg.ranges[idx]
            distance_px = distance_m/resolution

            #compute pixel in map:
            x = int(x_vehicle_map + distance_px * math.cos(angle_rad))
            y = int(y_vehicle_map + distance_px * math.sin(angle_rad))

            #if i == 0:
            #    print("laser pos", x,y)

            #make pixel black if within image bounds
            if 0 <= x < maxx and 0 <= y < maxy:
                pxmap[y][x] = 1.0

        end = time.time()
        #print(end-start)
        #print("veh pos: ", x_vehicle_map, y_vehicle_map, flush=True)
        #pyplot.imshow(self.__map.get_pixel_map())
        #pyplot.show()

    
    def parse_config(self, filename: str):
        f = open(filename, 'r')
        self.__config = yaml.safe_load(f)
        f.close()

    def compute_index_on_raceline(self, x_vehicle_map: int, y_vehicle_map: int, useCache=False) -> int:
                #check if we have lost tracking of the raceline
        if self.index_on_raceline > 0:
            diff_x = self.raceline.x[self.index_on_raceline]-x_vehicle_map
            diff_y = self.raceline.y[self.index_on_raceline]-y_vehicle_map
            len_diff = math.sqrt(diff_x*diff_x+diff_y*diff_y)

            if len_diff > 2.0:
                self.index_on_raceline = -1


        #find the point of the raceline where we currently are
        best_diff = 100000
        best_idx = self.index_on_raceline
        found_better_point = False
        for i in range(len(self.raceline.x)):
            idx = (self.index_on_raceline + i) % len(self.raceline.x)
            diff_x = self.raceline.x[idx]-x_vehicle_map
            diff_y = self.raceline.y[idx]-y_vehicle_map
            len_diff = math.sqrt(diff_x*diff_x+diff_y*diff_y)
            if len_diff < best_diff:
                #found better point!
                best_diff = len_diff
                best_idx = idx
                found_better_point = True
            else:
                if self.index_on_raceline > -1 and found_better_point:
                    #last time we found a better point - this time not:
                    # we can end the search as from now on, we will be moving away from the best point
                    #break
                    pass

        
        if useCache:
            self.index_on_raceline = best_idx

        return best_idx
    
    def compute_index_on_trajectory(self, trajectory: Trajectory, x_vehicle_map: int, y_vehicle_map: int, useCache=False, startIndex=0) -> int:

        #find the point of the raceline where we currently are
        best_diff = 100000
        best_idx = 0
        for idx in range(startIndex, len(trajectory.x)):
            diff_x = trajectory.x[idx]-x_vehicle_map
            diff_y = trajectory.y[idx]-y_vehicle_map
            len_diff = math.sqrt(diff_x*diff_x+diff_y*diff_y)
            if len_diff < best_diff:
                #found better point!
                best_diff = len_diff
                best_idx = idx
        
        return best_idx
    
    def __trajectory_from_controlpoints(self, xs, ys, num_samples, vehicle_width_in_map_pixels):
        sx,sy, _, cur, _ = pyspline.calc_2d_spline_interpolation(xs, ys, num=num_samples)
        s = Trajectory(sx, sy, self.__vehicle_description, self.__config["resolution"], curvature=cur, is_a_loop=False, leave_as_is=True)
        collision_index = self.__map.collision_at(s, vehicle_width_in_map_pixels)
        return s, collision_index

    def update_internal_vehicle_model(self, yaw_deg, speed, delta_t):
        delta_m = delta_t*speed
        self.vehicle_x   += math.cos(math.radians(yaw_deg)+self.vehicle_yaw)    * delta_m
        self.vehicle_y   += math.sin(math.radians(yaw_deg)+self.vehicle_yaw)    * delta_m
        self.vehicle_yaw += math.tan(math.radians(yaw_deg))/self.__vehicle_description.vehicle_length_m * delta_m

    #get internal pose (uses pose from localization if derivated too much)
    def get_vehicle_pose(self, x, y, yaw_rad):
        if self.vehicle_x is None:
            self.vehicle_x = x
            self.vehicle_y = y
            self.vehicle_yaw = yaw_rad
        else:
            if abs(self.vehicle_x -x) > 0.2 or abs(self.vehicle_y -y) > 0.2 or abs(self.vehicle_yaw -yaw_rad) > math.radians(5):
                self.vehicle_x = x
                self.vehicle_y = y
                self.vehicle_yaw = yaw_rad
        
        return self.vehicle_x, self.vehicle_y, self.vehicle_yaw


    # updates internal fine trajectory with current vehicle position and computes driving commands
    # DOES NOT REPAIR trajectorx in case of any obstacles.
    def compute_next_command(self, x_vehicle_map_m, y_vehicle_map_m, vehicle_yaw, vehicle_speed, delta_t):
        
        if self.__current_trajectory_fine is None or len(self.__current_trajectory_fine.x) < 3:
            return None, None

        #print(f"compute_next_command(self, {x_vehicle_map_m}, {y_vehicle_map_m}, {vehicle_yaw}, {vehicle_speed}, {delta_t})", flush=True)

        #internally, we compute everything in pixels - hence, we first need to convert from meters to pixels

        #convert from meters to pixels
        x_vehicle_map, y_vehicle_map = self.convert_from_meters_to_pixels(x_vehicle_map_m, y_vehicle_map_m)

        #initialize vehicle parameters, if called for the first time:
        if self.vehicle_x is None:
            self.vehicle_x = x_vehicle_map
            self.vehicle_y = y_vehicle_map
            self.vehicle_yaw = vehicle_yaw


        #consider internal model for position smoothing
        #x_vehicle_map, y_vehicle_map, vehicle_yaw = self.get_vehicle_pose(x_vehicle_map, y_vehicle_map, vehicle_yaw)
        
        #update fine trajectory to reflect vehicle position:
        index_on_fine_trajectory = self.compute_index_on_trajectory(self.__current_trajectory_fine, x_vehicle_map, y_vehicle_map, False, startIndex=0)
        #print(self.FigCounter, index_on_fine_trajectory, x_vehicle_map, y_vehicle_map, flush=True)
        #print(self.__current_trajectory_fine.x)
        #print(self.__current_trajectory_fine.y)
        if index_on_fine_trajectory >= len(self.__current_trajectory_fine.x)-1:
            print(f"error: current index of fine trajectory: {index_on_fine_trajectory} / {len(self.__current_trajectory_fine.x)}", flush=True)
            return None, None

        self.__current_trajectory_fine.x = self.__current_trajectory_fine.x[index_on_fine_trajectory:]
        self.__current_trajectory_fine.y = self.__current_trajectory_fine.y[index_on_fine_trajectory:]
        #self.__current_trajectory_fine.x[0] = x_vehicle_map
        #self.__current_trajectory_fine.y[0] = y_vehicle_map




        s = self.__current_trajectory_fine
        #compute velocities
        s.compute_velocity_profile()
        s.velocity_profile[0] = vehicle_speed
        s.adjust_velocity_to_acceleration_forwards_pass(0)
        s.adjust_velocity_to_acceleration_backwards_pass(1)
        
        
        #get steering angle from trajectory
        target_index = 2    #we try heading towards the 2nd point of the fine trajectory
        input_angle = 1000
        angle = 0.0

        #if vehicle speed is high, it might help to aim for further away points to stabelize the trajectory:
        if vehicle_speed > 3.0:
            target_index = min( int(vehicle_speed*2), len(s.x)-1)



            
        dx = s.x[target_index % len(s.x)] - x_vehicle_map
        dy = s.y[target_index % len(s.x)] - y_vehicle_map
        l = math.sqrt(dx**2 + dy**2)
        angle = vehicle_yaw
        if l != 0.0:
            angle = math.atan2(dy/l, dx/l) - vehicle_yaw #TODO: this is wrong as we dont consider vehicle length
            #angle = 2*dy/(l**2)
        

        #print(f"angle to s({target_index}) is {math.degrees(angle)}", flush=True)
        
        #unwind angle into input_angle
        input_angle = math.degrees(angle) % 360
        while input_angle < -180:
            input_angle += 360
        while input_angle > 180:
            input_angle -= 360

    
        input_angle = min(max(input_angle, self.__vehicle_description.vehicle_min_steering_angle), self.__vehicle_description.vehicle_max_steering_angle)
        angle = math.radians(input_angle)

        #make sure we are not too fast to turn:
        radius_m = 10000
        if angle != 0:
            radius_m = abs(s.get_vehicle_description().vehicle_length_m / math.sin(angle))
        max_velocity_possible = math.sqrt((s.get_vehicle_description().haftreibung * radius_m) / s.get_vehicle_description().vehicle_mass ) #due to antipetal force

        desired_speed = min(s.velocity_profile[target_index], max_velocity_possible)

        self.update_internal_vehicle_model(speed=desired_speed, yaw_deg=math.degrees(angle), delta_t=delta_t)

        return desired_speed, math.degrees(angle)
    

    def convert_from_meters_to_pixels(self, x_vehicle_map_m, y_vehicle_map_m):
        #remove offset from vehicle pos
        origin = self.__map.get_origin()
        x_vehicle_map_m, y_vehicle_map_m = x_vehicle_map_m - origin[0], y_vehicle_map_m - origin[1]

        #convert from meters to pixels
        x_vehicle_map, y_vehicle_map = x_vehicle_map_m /self.__config["resolution"], y_vehicle_map_m / self.__config["resolution"]

        #swap y-axis
        max_y = self.__map.get_map_size_pixels()[0]
        y_vehicle_map = (max_y - y_vehicle_map)

        return x_vehicle_map, y_vehicle_map
    

    def set_fine_trajectory(self, xs, ys, curvature, velocities):
        #convert from meters to pixels

        self.__current_trajectory_fine = Trajectory(xs, ys, self.__vehicle_description, self.__config["resolution"], curvature=curvature, is_a_loop=False, leave_as_is=True)
        if velocities is not None:
            self.__current_trajectory_fine.velocity_profile = velocities
        
        

    # repaires current trajectory in case of a future collision
    def trajectory_collision_resolution(self, x_vehicle_map_m, y_vehicle_map_m, vehicle_yaw, vehicle_speed, delta_t):

        #print(f"compute_next_command(self, {x_vehicle_map_m}, {y_vehicle_map_m}, {vehicle_yaw}, {vehicle_speed}, {delta_t})", flush=True)

        #internally, we compute everything in pixels - hence, we first need to convert from meters to pixels

        #convert from meters to pixels
        x_vehicle_map, y_vehicle_map = self.convert_from_meters_to_pixels(x_vehicle_map_m, y_vehicle_map_m)


        #initialize vehicle parameters, if called for the first time:
        if self.vehicle_x is None:
            self.vehicle_x = x_vehicle_map
            self.vehicle_y = y_vehicle_map
            self.vehicle_yaw = vehicle_yaw


        #consider internal model for position smoothing
        #x_vehicle_map, y_vehicle_map, vehicle_yaw = self.get_vehicle_pose(x_vehicle_map, y_vehicle_map, vehicle_yaw)

        vehicleIndexOnRaceline = self.compute_index_on_raceline(x_vehicle_map, y_vehicle_map, useCache=True)

        #take next lookahead meters from raceline as control points:
        lookahead_m = self.__lookahead_m
        raceline_length_m = self.raceline.get_length() * self.__config["resolution"]
        num_points = lookahead_m/raceline_length_m * len(self.raceline.x) #number of points in the coarse trajectory
        vehicle_width_in_map_pixels = math.ceil(self.__vehicle_description.vehicle_width_m / self.__config['resolution'])

        num_samples = int(lookahead_m * self.__points_per_meter) #points per meter in the fine trajectory

        #initialize current trajectory, if not happend yet:
        if self.__current_trajectory_coarse is None:
        
            from_idx = vehicleIndexOnRaceline
            to_idx = int(vehicleIndexOnRaceline+num_points) % len(self.raceline.x)
            xs = []
            ys = []
            if to_idx > from_idx:
                xs = self.raceline.x[from_idx:to_idx]
                ys = self.raceline.y[from_idx:to_idx]
            else:
                xs = self.raceline.x[from_idx:] + self.raceline.x[0: to_idx]
                ys = self.raceline.x[from_idx:] + self.raceline.x[0: to_idx]
            cx,cy, _, _, _ = pyspline.calc_2d_spline_interpolation(xs, ys, num=lookahead_m*2) #every 0.5m a controlpoint

            self.__last_added_raceline_point = self.compute_index_on_trajectory(
                Trajectory(self.controlpoints_raceline_x, 
                            self.controlpoints_raceline_y, 
                            self.__vehicle_description, 
                            self.__config["resolution"]),
                cx[-1], 
                cy[-1],
                False)+2
            
            #replace first point with current vehicle position
            cx[0] = x_vehicle_map
            cy[0] = y_vehicle_map

            #coarse placement of control points for optimization
            self.__current_trajectory_coarse = Trajectory(cx, cy, self.__vehicle_description, self.__config["resolution"], is_a_loop=False)

            #resample raceline and check where it collides --> move corresponding controlpoints out of the way
            self.__current_trajectory_fine, collision_index = self.__trajectory_from_controlpoints(xs, ys, num_samples, vehicle_width_in_map_pixels)
        else:
            #update trajectory with current location
            index_on_coarse_trajectory = self.compute_index_on_trajectory(self.__current_trajectory_coarse, x_vehicle_map, y_vehicle_map, False)
            self.__current_trajectory_coarse.x = self.__current_trajectory_coarse.x[index_on_coarse_trajectory:]
            self.__current_trajectory_coarse.y = self.__current_trajectory_coarse.y[index_on_coarse_trajectory:]
            self.__current_trajectory_coarse.x[0] = x_vehicle_map
            self.__current_trajectory_coarse.y[0] = y_vehicle_map


            #extend trajectory to lookahead_m meters, if necessary
            self.__current_trajectory_coarse.length = None #force re-calculation of length
            current_len = self.__current_trajectory_coarse.get_length() * self.__config["resolution"]
            while current_len < lookahead_m:
                self.__current_trajectory_coarse.x.append( self.controlpoints_raceline_x[(self.__last_added_raceline_point) % len(self.controlpoints_raceline_x)] )
                self.__current_trajectory_coarse.y.append( self.controlpoints_raceline_y[(self.__last_added_raceline_point) % len(self.controlpoints_raceline_x)] )
                self.__current_trajectory_coarse.length = None #force re-calculation of length
                self.__last_added_raceline_point = (self.__last_added_raceline_point+1) % len(self.controlpoints_raceline_x)
                current_len = self.__current_trajectory_coarse.get_length() * self.__config["resolution"]

                #re-create fine trajectory from coarse trajectory
                self.__current_trajectory_fine, collision_index = self.__trajectory_from_controlpoints(self.__current_trajectory_coarse.x, self.__current_trajectory_coarse.y, num_samples, vehicle_width_in_map_pixels)
            


        #update fine trajectory to reflect vehicle position:
        index_on_fine_trajectory = self.compute_index_on_trajectory(self.__current_trajectory_fine, x_vehicle_map, y_vehicle_map, False, startIndex=0)

        self.__current_trajectory_fine.x = self.__current_trajectory_fine.x[index_on_fine_trajectory:]
        self.__current_trajectory_fine.y = self.__current_trajectory_fine.y[index_on_fine_trajectory:]


        #check if we are heading into a collision:
        _, collision_index = self.__trajectory_from_controlpoints(self.__current_trajectory_coarse.x, self.__current_trajectory_coarse.y, num_samples, vehicle_width_in_map_pixels)


        #debug
        """if collision_index >= 0:
            print(f"collision index: {collision_index} (max: {len(self.__current_trajectory_coarse.x)})")
            self.__current_trajectory_fine.compute_velocity_profile()
            pyplot.imshow(self.__map.get_pixel_map())
            pyplot.scatter(self.__current_trajectory_fine.x,self.__current_trajectory_fine.y, c=self.__current_trajectory_fine.velocity_profile, linewidth=1, cmap=pyplot.cm.coolwarm)
            pyplot.colorbar()
            #pyplot.scatter(f.x,f.y, c=f.velocity_profile, linewidth=1, cmap=pyplot.cm.coolwarm)
            pyplot.scatter(self.__current_trajectory_fine.x[collision_index], self.__current_trajectory_fine.y[collision_index], c="black", linewidth=0.5)
            pyplot.savefig(f"/tmp/mpc{self.FigCounter}.png")
            pyplot.figure().clear()
            pyplot.clf()
            pyplot.close('all')
            self.FigCounter += 1
            #end debug
        """

        #try to repair trajectory if we have a collision:
        if collision_index >= 0:
            count = 0
            max_tries = self.__candidate_count
            use_normal_vector = True

            smallestChange = 10000
            smallestCoarse, smallestFine = None, None
            while count < max_tries:

                if count > max_tries/2:
                    use_normal_vector = False
                #random changes to the coarse trajectory:
                t1 = time.time()
                c = Trajectory(self.__current_trajectory_coarse.x.copy(), self.__current_trajectory_coarse.y.copy(), self.__vehicle_description, self.__config["resolution"], is_a_loop=False, leave_as_is=True)
                t2  = time.time()
                change = c.random_changes(max_change_px=1.0/self.__config["resolution"],
                                            num_changes=1,
                                            map=self.__map,
                                            num_ctrl_points=num_samples,
                                            apply_smoothing=False,
                                            idx=int(collision_index/(len(self.__current_trajectory_fine.x)/len(self.__current_trajectory_coarse.x))),
                                            use_normal_vector=use_normal_vector,
                                            num_adjacent=1)
                t3  = time.time()
                #construct fine trajectory from coarse:
                f, collision_index = self.__trajectory_from_controlpoints(c.x, c.y, num_samples, vehicle_width_in_map_pixels)
                t4 = time.time()
                #collision_index = self.__map.collision_at(f, vehicle_width_in_map_pixels)
                t5 = time.time()

                dt1 = (t2-t1)*1000.0
                dt2 = (t3-t2)*1000.0
                dt3 = (t4-t3)*1000.0
                dt4 = (t5-t4)*1000.0
                #print(dt1, dt2, dt3, dt4)

                if collision_index < 0 and change < smallestChange:
                    smallestChange = change
                    smallestCoarse = c
                    smallestFine = f

                count += 1

            if smallestFine is not None:
                self.__current_trajectory_fine = smallestFine
                self.__current_trajectory_coarse = smallestCoarse


        if collision_index >= 0:
            #could not repair trajectory.
            return False
        

        #try to optimize current trajectory:
        """self.__current_trajectory_fine.compute_velocity_profile()
        self.__current_trajectory_fine.velocity_profile[0] = vehicle_speed
        self.__current_trajectory_fine.adjust_velocity_to_acceleration_forwards_pass(0)
        self.__current_trajectory_fine.adjust_velocity_to_acceleration_backwards_pass(1)
        bestTime = self.__current_trajectory_fine.get_laptime(num_samples=100)

        for i in range(0,0): #disable
            c = self.__current_trajectory_coarse.copy()
            c.random_changes(max_change_px=0.25/self.__config["resolution"],
                                        num_changes=1,
                                        map=self.__map,
                                        num_ctrl_points=len(c.x), apply_smoothing=False,
                                        idx=int(random.random() * (len(c.x) - 5) + 5))
            
            f, collision_index = self.__trajectory_from_controlpoints(c.x, c.y, num_samples, vehicle_width_in_map_pixels)
            if collision_index < 0:
                f.compute_velocity_profile()
                f.velocity_profile[0] = vehicle_speed
                f.adjust_velocity_to_acceleration_forwards_pass(0)
                f.adjust_velocity_to_acceleration_backwards_pass(1)

                if f.get_laptime(num_samples=100) < bestTime:      #TODO: closest to raceline would be better
                    self.__current_trajectory_fine = f
                    self.__current_trajectory_coarse = c
                    bestTime = f.get_laptime(num_samples=100)

        
        """
        s = self.__current_trajectory_fine
        #compute velocities
        s.compute_velocity_profile()
        s.velocity_profile[0] = vehicle_speed
        s.adjust_velocity_to_acceleration_forwards_pass(0)
        s.adjust_velocity_to_acceleration_backwards_pass(1)

        
        #plot debug images
        """
        pyplot.imshow(self.__map.get_pixel_map())
        pyplot.scatter(s.x,s.y, c=s.velocity_profile, linewidth=1, cmap=pyplot.cm.coolwarm)
        pyplot.colorbar()
        #pyplot.scatter(f.x,f.y, c=f.velocity_profile, linewidth=1, cmap=pyplot.cm.coolwarm)
        pyplot.scatter(x_vehicle_map, y_vehicle_map, c="black", linewidth=0.5)
        pyplot.savefig(f"/tmp/mpc{self.FigCounter}.png")
        pyplot.figure().clear()
        pyplot.clf()
        pyplot.close('all')
        self.FigCounter += 1
        """

        #could repair trajectory
        return True



    
    def plotTrajectory(self, xs, ys, speeds):
        #debug output
        pyplot.figure().clf()
        pyplot.close('all')
        pyplot.imshow(self.__map.get_pixel_map())
        pyplot.scatter(self.raceline.x, self.raceline.y, linewidth=0.5)

        

        pyplot.scatter([x/self.__config["resolution"] for x in xs], [y/self.__config["resolution"] for y in ys], c=speeds, linewidth=1, cmap=pyplot.cm.coolwarm)
        pyplot.colorbar()


        pyplot.show()


if __name__ == "__main__":
    print("Testcode for LaserScan in MPC")
    #test laser:
    class LaserScan:
        def __init__(self):
            self.ranges = []
            self.angle_increment = math.radians(360/1024)
    
    #make a spiral
    msg = LaserScan()
    msg.ranges = [0 for i in range(1024)]
    for i in range(0, 1024):
        msg.ranges[(i+512)%1024] =  i/1024 * 3.0 + 0.1
    

    #vehicle description
    vehicle_length_m            = 0.5 
    haftreibung                 = 6.0     #kg force to move standing vehicle in lateral direction
    vehicle_mass_kg             = 3.0
    vehicle_width_m             = 0.3     #half width is minimum distance to any wall at any time
    vehicle_acceleration_mss    = 7.0     #vehicle acceleration in meters/sec/sec
    vehicle_deceleration_mss    = 3.0     #vehicle deceleration in meters/sec/sec
    vehicle_max_steering_angle = 30.0
    vd = VehicleDescription(haftreibung, vehicle_width_m,  vehicle_mass_kg, vehicle_acceleration_mss, vehicle_deceleration_mss, vehicle_length_m, -vehicle_max_steering_angle, vehicle_max_steering_angle)

    #initial vehicle state
    vehicle_x = 6.0
    vehicle_y = 2.0
    vehicle_yaw = 0.0
    vehicle_speed = 2.0


    mpc = MPCController(mapconfigfile="/Users/wette/Documents/FHBielefeld/eigeneVorlesungen/F110/repositories/wette_racecar_ws/minden_obs.yaml",
                              raceline="/Users/wette/Documents/FHBielefeld/eigeneVorlesungen/F110/repositories/f1tenth_raceline/my_map_raceline.csv",
                              vehicle_description=vd)
    
    mpc.compute_next_command(vehicle_x, 
                            vehicle_y, 
                            vehicle_yaw, 
                            vehicle_speed,
                            delta_t=0.05)
    
    mpc.callback_new_laser(msg)

    mpc.compute_next_command(vehicle_x+1, 
                            vehicle_y+1, 
                            vehicle_yaw, 
                            vehicle_speed,
                            delta_t=0.05)
    mpc.callback_new_laser(msg)

    mpc.plotTrajectory([vehicle_x], [vehicle_y], [vehicle_speed])
    

