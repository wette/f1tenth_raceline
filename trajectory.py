import random
import math
import pycubicspline.pycubicspline as pyspline

class Trajectory:
    def __init__(self, x: list, y: list, curvature=None):
        self.x = x
        self.y = y
        self.curvature = curvature
        self.length = None
        self.laptime = None

        if self.curvature is None:
            self.x, self.y, _, self.curvature, _ = pyspline.calc_2d_spline_interpolation(self.x, self.y, num=len(self.y))
    
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
    
    def get_laptime(self) -> int:
        if self.laptime is not None:
            return self.laptime
        
        haftreibung = 0.4
        
        #compute length of spline and divide by max. curvature speed --> time
        self.laptime = 0.0
        for i in range(0, len(self.x)-1):
            x2 = self.x[i+1] - self.x[i]
            x2 *= x2
            y2 = self.y[i+1] - self.y[i]
            y2 *= y2

            length = math.sqrt( x2 + y2 )
            max_speed = math.sqrt(haftreibung/(1.0-haftreibung) * 9.8 * abs(1.0/self.curvature[i]))
            if max_speed < 0.0001:
                continue
            self.laptime += length/max_speed

        #add length from last back to first.
        x2 = self.x[-1] - self.x[0]
        x2 *= x2
        y2 = self.y[-1] - self.y[0]
        y2 *= y2
        
        length = math.sqrt( x2 + y2 )
        max_speed = math.sqrt(haftreibung/(1.0-haftreibung) * 9.8 * abs(1.0/self.curvature[i]))
        self.laptime += length/max_speed

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

    def random_combination(self, other_trajectory: 'Trajectory'):
        start_idx = random.randint(0, len(self.x)-1)
        end_idx   = random.randint(start_idx, len(self.x)-1)

        for i in range(start_idx, end_idx):
            self.x[i] = other_trajectory.x[i]
            self.y[i] = other_trajectory.y[i]

        #apply spline smoothing
        self.x, self.y, _, self.curvature, _ = pyspline.calc_2d_spline_interpolation(self.x, self.y, num=len(self.y))
        self.length = None

    def copy(self) -> 'Trajectory':
        if self.curvature is not None:
            return Trajectory(self.x[:], self.y[:], self.curvature[:])
        else:
            return Trajectory(self.x[:], self.y[:])
