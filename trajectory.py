import random
import math
import pycubicspline.pycubicspline as pyspline

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

        #if self.curvature is None:
        self.x, self.y, _, self.curvature, _ = pyspline.calc_2d_spline_interpolation(self.x, self.y, num=len(self.y))

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
        copyVeloProfile = self.velocity_profile.copy()

        for numProcessed in range(len(copyVeloProfile)):
            #find lowest speed and go backwards
            i = copyVeloProfile.index(min(copyVeloProfile))
            copyVeloProfile[i] = math.inf    #to not find this point again
            origi = i
            velocity = self.velocity_profile[i] #speed at this waypoint (m/s) #TODO: this is not correct: must be average speed, not minimum!!
            xd = self.x[i-1] - self.x[i]
            yd = self.y[i-1] - self.y[i]
            way = math.sqrt(xd*xd+yd*yd)*self.resolution #distance from last to this waypoint (m)
            time = way/velocity # (s)
            while self.velocity_profile[(i-1) % len(self.velocity_profile)]-self.vehicle_deceleration_mss*time > self.velocity_profile[i]:
                self.velocity_profile[(i-1) % len(self.velocity_profile)] = self.velocity_profile[i]+self.vehicle_deceleration_mss*time
                i = (i - 1) % len(self.velocity_profile)

            #do the same thing forwards, to compute realistic velocity based on the vehicle acceleration
            i = origi
            velocity = self.velocity_profile[i] #speed at this waypoint (m/s) #TODO: this is not correct: must be average speed, not minimum!!
            xd = self.x[i] - self.x[(i+1) % len(self.velocity_profile)]
            yd = self.y[i] - self.y[(i+1) % len(self.velocity_profile)]
            way = math.sqrt(xd*xd+yd*yd)*self.resolution #distance from this to next waypoint (m)
            time = way/velocity # (s)
            while self.velocity_profile[(i+1) % len(self.velocity_profile)]-self.vehicle_acceleration_mss*time > self.velocity_profile[i]:
                self.velocity_profile[(i+1) % len(self.velocity_profile)] = self.velocity_profile[i]+self.vehicle_acceleration_mss*time
                i = (i + 1) % len(self.velocity_profile)
        
    
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
        x2 = self.x[-1] - self.x[0]
        x2 *= x2
        y2 = self.y[-1] - self.y[0]
        y2 *= y2
        
        length = math.sqrt( x2 + y2 )
        speed = (self.velocity_profile[i] + self.velocity_profile[i+1])/2.0
        self.laptime += length/speed

        return self.laptime


    def random_changes(self, max_change_px: float, num_changes: int, map: list):
        """randomly change the trajectory"""
        for i in range(num_changes):
            idx = random.randint(0, len(self.x)-2) #exclude last point
            x = self.x[idx]
            y = self.y[idx]
            #test different changes - keep the first one which is in free space
            while True:
                change_x = max_change_px - random.random()*max_change_px*2.0
                change_y = max_change_px - random.random()*max_change_px*2.0

                self.x[idx] = x + change_x
                self.y[idx] = y + change_y
                #if first point is moved, last point needs to move, too!
                if idx == 0:
                    self.x[-1] = self.x[idx]
                    self.y[-1] = self.x[idx]

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

        #apply spline smoothing
        self.x, self.y, _, self.curvature, _ = pyspline.calc_2d_spline_interpolation(self.x, self.y, num=len(self.y))
        self.length = None
        self.laptime = None

    def copy(self) -> 'Trajectory':
        if self.curvature is not None:
            return Trajectory(self.x[:], self.y[:], self.haftreibung, self.vehicle_width_m, self.vehicle_acceleration_mss, self.vehicle_deceleration_mss, self.resolution, self.curvature[:])
        else:
            return Trajectory(self.x[:], self.y[:], self.haftreibung, self.vehicle_width_m, self.vehicle_acceleration_mss, self.vehicle_deceleration_mss, self.resolution)
