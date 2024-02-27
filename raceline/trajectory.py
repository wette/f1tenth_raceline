import random
import math
import pycubicspline.pycubicspline as pyspline
import numpy

class Trajectory:
    def __init__(self, x: list, y: list, haftreibung: float, vehicle_width_m: float, 
                 vehicle_acceleration_mss: float, vehicle_deceleration_mss: float, resolution: float, curvature=None):
        self.x = x
        self.y = y
        self.haftreibung = haftreibung
        self.curvature = curvature
        self.vehicle_width_m = vehicle_width_m
        self.vehicle_acceleration_mss = vehicle_acceleration_mss
        self.vehicle_deceleration_mss = vehicle_deceleration_mss
        self.resolution = resolution
        self.length = None
        self.laptime = None

        self.velocity_profile = None #for each point the velocity of the vehicle in meters/second

        self.do_forwards_pass = False #gives straighter lines as there is more emphasis on acceleration

        self.x, self.y, _, self.curvature, _ = pyspline.calc_2d_spline_interpolation(self.x, self.y, num=len(self.y))
        self.remove_overlapping_points()

    def remove_overlapping_points(self):
        """remove points at the end of the spline that overlap with points at the beginning at the spline."""
        eps = 1
        toRemove = []
        for i in range(len(self.x)-1, 1, -1):
            found = False
            for j in range(1, len(self.x)):
                if i == j:
                    continue
                dx = self.x[i]-self.x[j]
                dy = self.y[i]-self.y[j]
                l2 = math.sqrt(dx*dx+dy*dy)
                if(l2 < eps):
                    #same point - remove.
                    toRemove.append(i)
                    found = True
            if not found:
                break

        for i in toRemove:
            self.x.pop(i)
            self.y.pop(i)
            self.curvature.pop(i)


    def adjust_velocity_to_acceleration_backwards_pass(self, i: int):
        """adjust velocity profile to be physically possible"""
        while True:
            velocity = self.velocity_profile[i] #speed at this waypoint (m/s) #TODO: this is not correct: must be average speed, not minimum!!
            xd = self.x[i-1] - self.x[i]
            yd = self.y[i-1] - self.y[i]
            way = math.sqrt(xd*xd+yd*yd)*self.resolution #distance from last to this waypoint (m)
            time = way/velocity # (s)
            if self.velocity_profile[(i-1) % len(self.velocity_profile)]-self.vehicle_deceleration_mss*time > self.velocity_profile[i]:
                self.velocity_profile[(i-1) % len(self.velocity_profile)] = self.velocity_profile[i]+self.vehicle_deceleration_mss*time
                i = (i - 1) % len(self.velocity_profile)
            else:
                break

    def adjust_velocity_to_acceleration_forwards_pass(self, i: int):
        """adjust velocity profile to be physically possible"""
        while True:
            velocity = self.velocity_profile[i] #speed at this waypoint (m/s) #TODO: this is not correct: must be average speed, not minimum!!
            xd = self.x[i] - self.x[(i+1) % len(self.velocity_profile)]
            yd = self.y[i] - self.y[(i+1) % len(self.velocity_profile)]
            way = math.sqrt(xd*xd+yd*yd)*self.resolution #distance from this to next waypoint (m)
            time = way/velocity # (s)
            if self.velocity_profile[(i+1) % len(self.velocity_profile)] > self.velocity_profile[i] + self.vehicle_acceleration_mss*time:
                self.velocity_profile[(i+1) % len(self.velocity_profile)] = self.velocity_profile[i]+self.vehicle_acceleration_mss*time
                i = (i + 1) % len(self.velocity_profile)
            else:
                break

        

    def compute_velocity_profile(self):
        """for each point on the spline, compute the max. possible velocity given a certain traction"""
        #formula taken from https://www.johannes-strommer.com/fahrzeug-formeln/geschwindigkeit-in-kurven/

        #This assumes infinite acceleration/deceleration and gives us the max. possible cornering speeds.
        self.velocity_profile = []
        for i in range(0, len(self.x)):
            max_velocity = math.sqrt(self.haftreibung/(1.0-self.haftreibung) * 9.8 * abs(1.0/self.curvature[i]))
            self.velocity_profile.append(max_velocity)

        # we now need to resitict speed on the straights such that we have enough braking power to decelerate
        # to this end, find the lowest speed, and go back over the trajectory and decrease speeds if nececcary
        processedList = []

        for numProcessed in range(len(self.x)):
            #find lowest speed and go backwards
            sortedIds = list(numpy.argsort(self.velocity_profile))
            for i in processedList:
                sortedIds.remove(i)

            i = sortedIds[0]

            #go backwards over the trajectory and decrease speeds if nececcary
            self.adjust_velocity_to_acceleration_backwards_pass(i)

            #do the same thing forwards, to compute realistic velocity based on the vehicle acceleration
            if self.do_forwards_pass:
                self.adjust_velocity_to_acceleration_forwards_pass(i)

            processedList.append(i) #point processed
        
    
    def get_length(self) -> int:
        if self.length is not None:
            return self.length
        
        #compute length of spline
        self.length = 0.0
        for i in range(0, len(self.x)-1):
            x2 = self.x[i+1] - self.x[i]
            x2 *= x2
            y2 = self.y[i+1] - self.y[i]
            y2 *= y2
            self.length += math.sqrt( x2 + y2 )

        #add length from last back to first.
        x2 = self.x[-1] - self.x[0]
        x2 *= x2
        y2 = self.y[-1] - self.y[0]
        y2 *= y2
        self.length += math.sqrt( x2 + y2 )

        return self.length
    
    def get_laptime_old(self) -> int:
        if self.laptime is not None:
            return self.laptime
        
        #compute length of spline and divide by max. curvature speed --> time
        self.laptime = 0.0
        for i in range(0, len(self.x)-1):
            x2 = self.x[i+1] - self.x[i]
            x2 *= x2
            y2 = self.y[i+1] - self.y[i]
            y2 *= y2

            length = math.sqrt( x2 + y2 )
            max_speed = math.sqrt(self.haftreibung/(1.0-self.haftreibung) * 9.8 * abs(1.0/self.curvature[i]))
            if max_speed < 0.0001:
                continue
            self.laptime += length/max_speed

        #add length from last back to first.
        x2 = self.x[-1] - self.x[0]
        x2 *= x2
        y2 = self.y[-1] - self.y[0]
        y2 *= y2
        
        length = math.sqrt( x2 + y2 )
        max_speed = math.sqrt(self.haftreibung/(1.0-self.haftreibung) * 9.8 * abs(1.0/self.curvature[i]))
        self.laptime += length/max_speed

        return self.laptime
    
    def get_laptime(self) -> int:
        if self.laptime is not None:
            return self.laptime
        
        self.compute_velocity_profile()
        #compute length of spline and divide by velocity --> time
        self.laptime = 0.0
        for i in range(0, len(self.x)-1):
            x2 = self.x[i+1] - self.x[i]
            x2 *= x2
            y2 = self.y[i+1] - self.y[i]
            y2 *= y2

            length = math.sqrt( x2 + y2 )
            speed = (self.velocity_profile[i] + self.velocity_profile[i+1])/2.0
            if speed < 0.0001:
                continue
            self.laptime += length/speed

        #add length from last back to first.
        #x2 = self.x[-1] - self.x[0]
        #x2 *= x2
        #y2 = self.y[-1] - self.y[0]
        #y2 *= y2
        #
        #length = math.sqrt( x2 + y2 )
        #speed = (self.velocity_profile[i] + self.velocity_profile[i+1])/2.0
        #self.laptime += length/speed

        return self.laptime


    def compute_normal_vector(self, idx: int):
        """compute the normalized (orthogonal) normal vector for point with index idx"""
        prev_x = self.x[(idx-1) % len(self.x)]
        prev_y = self.y[(idx-1) % len(self.y)]
        next_x = self.x[(idx+1) % len(self.x)]
        next_y = self.y[(idx+1) % len(self.y)]

        dx = next_x-prev_x
        dy = next_y-prev_y

        orto_dx = -dy
        orto_dy = dx

        length = math.sqrt(orto_dx*orto_dx + orto_dy*orto_dy)

        return orto_dx/length, orto_dy/length
    
    def compute_random_vector(self):
        """compute a normalized random vector"""
        dx = 1.0-random.random()*2.0
        dy = 1.0-random.random()*2.0

        length = math.sqrt(dx*dx + dy*dy)

        return dx/length, dy/length

    def random_changes(self, max_change_px: float, num_changes: int, map: list):
        """randomly change the trajectory"""
        for i in range(num_changes):
            idx = random.randint(0, len(self.x)-2) #exclude last point
            x = self.x[idx]
            y = self.y[idx]
            #test different changes - keep the first one which is in free space
            while True:
                normalx, normaly = self.compute_random_vector() #self.compute_normal_vector(idx)
                change = random.random()*max_change_px

                self.x[idx] = x + change*normalx
                self.y[idx] = y + change*normaly
                #if first point is moved, last point needs to move, too!
                if idx == 0:
                    self.x[len(self.x)-1] = self.x[idx]
                    self.y[len(self.y)-1] = self.y[idx]
                    

                if map[ int(self.y[idx]) ][ int(self.x[idx]) ] == 0.0:
                    break

        #apply spline smoothing
        self.x, self.y, _, self.curvature, _ = pyspline.calc_2d_spline_interpolation(self.x, self.y, num=len(self.y))
        self.length = None
        self.laptime = None

    def random_combination(self, other_trajectory: 'Trajectory'):
        start_idx = random.randint(0, len(self.x)-1)
        end_idx   = random.randint(start_idx, len(self.x)-1)

        for i in range(start_idx, end_idx):
            self.x[i] = other_trajectory.x[i]
            self.y[i] = other_trajectory.y[i]

        #make sure last and first point are the same:
        self.x[0] = self.x[len(self.x)-1]
        self.y[0] = self.y[len(self.y)-1]

        #apply spline smoothing
        self.x, self.y, _, self.curvature, _ = pyspline.calc_2d_spline_interpolation(self.x, self.y, num=len(self.y))
        self.length = None
        self.laptime = None

    def safe_trajectory_to_file(self, map: 'Map', file: str, num_points: int):
        """ write computed trajectory to file """

        f = open(file, "w")
        f.write("x\ty\tvelocity[m]\n")

        #add second point of spline to end such that the two ends of the spline form a continuous curve
        x,y, _, _, _ = pyspline.calc_2d_spline_interpolation(self.x + [self.x[1]], self.y + [self.y[1]], num=num_points)

        trajectory = Trajectory(x, y, self.haftreibung, self.vehicle_width_m, self.vehicle_acceleration_mss, self.vehicle_deceleration_mss, self.resolution)
        trajectory.do_forwards_pass = True
        trajectory.compute_velocity_profile()

        max_x, max_y = map.get_map_size_pixels()
        resolution = map.get_resolution()
        origin = map.get_origin()


        for i in range(len(trajectory.x)):
            x_px = trajectory.x[i]
            y_px = trajectory.y[i]

            #transform pixel to coordinates - y axis needs to be swapped around!
            x = x_px * resolution + origin[0]
            y = (max_x - y_px) * resolution + origin[1]

            f.write(f"{x}\t{y}\t{trajectory.velocity_profile[i]}\n")

        f.close()

    def load_trajectory_from_file(self, file: str):
        f = open(file, "r")
        f.readline() #ommit first line

        self.x = []
        self.y = []
        self.velocity_profile = []

        for line in f.readlines():
            tokens = line.split("\t")
            self.x.append(float(tokens[0]))
            self.y.append(float(tokens[1]))
            self.velocity_profile.append(float(tokens[2]))

        f.close()


    def copy(self) -> 'Trajectory':
        t = None
        if self.curvature is not None:
            t = Trajectory(self.x[:], self.y[:], self.haftreibung, self.vehicle_width_m, self.vehicle_acceleration_mss, self.vehicle_deceleration_mss, self.resolution, self.curvature[:])
        else:
            t = Trajectory(self.x[:], self.y[:], self.haftreibung, self.vehicle_width_m, self.vehicle_acceleration_mss, self.vehicle_deceleration_mss, self.resolution)

        t.do_forwards_pass = self.do_forwards_pass

        return t
