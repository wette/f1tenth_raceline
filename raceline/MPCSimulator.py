
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__)))

import math
import time
import random

from trajectory import VehicleDescription
from matplotlib import pyplot
from mpcController import MPCController

def main():

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
    vehicle_speed = 1.0


    mpc = MPCController(mapconfigfile="/Users/wette/Documents/FHBielefeld/eigeneVorlesungen/F110/repositories/wette_racecar_ws/minden_obs.yaml",
                              raceline="/Users/wette/Documents/FHBielefeld/eigeneVorlesungen/F110/repositories/f1tenth_raceline/my_map_raceline.csv",
                              vehicle_description=vd)


    #simulation parameters
    localization_error = [0, 0, 0]
    max_error_m = 0.1
    error_change_rate_m = 0.02
    error_change_rate_deg = 0.2
    max_yaw_error = 2
    delta_t = 0.05 #simulation step
    total_simulation_time_s = 17.0 * 1

    #keep track of simulation
    xs = []
    ys = []
    speeds = []
    oversteer = []
    error_x = []
    error_y = []
    error_yaw = []
    computation_times = []


    total_steps = int(total_simulation_time_s/delta_t)
    for i in range(total_steps):

        #update localization error
        localization_error[0] += random.random()*2*error_change_rate_m - error_change_rate_m
        localization_error[1] += random.random()*2*error_change_rate_m - error_change_rate_m
        localization_error[0]  = max(min(localization_error[0], max_error_m), -max_error_m)
        localization_error[1]  = max(min(localization_error[1], max_error_m), -max_error_m)

        localization_error[2] += random.random()*2*error_change_rate_deg - error_change_rate_deg
        localization_error[2]  = max(min(localization_error[2], max_yaw_error), -max_yaw_error)

        error_x.append(   localization_error[0] )
        error_y.append(   localization_error[1] )
        error_yaw.append( localization_error[2] )

        if vehicle_speed < 0.01:
            vehicle_speed = 1.0

        start = time.time()
        input_speed, input_angle = mpc.compute_next_command(vehicle_x   + localization_error[0], 
                                                                  vehicle_y   + localization_error[1], 
                                                                  vehicle_yaw + math.radians(localization_error[2]), 
                                                                  vehicle_speed,
                                                                  delta_t)
        end = time.time()

        computation_times.append(end-start)

        #restrict steering angle (unwind, first)
        input_angle = input_angle % 360
        while input_angle < -180:
            input_angle += 360
        while input_angle > 180:
            input_angle -= 360

        input_angle = max(-vehicle_max_steering_angle, min(input_angle, vehicle_max_steering_angle) )

        #check if desired speed is feasable due to limits of decelleration
        min_speed = vehicle_speed - delta_t * vehicle_deceleration_mss
        max_speed = vehicle_speed + delta_t * vehicle_acceleration_mss
        input_speed = min(max(input_speed, min_speed), max_speed)

        #calculate antipetal force to estimate oversteering
        radius_m = 10000
        if input_angle != 0:
            radius_m = abs(vehicle_length_m / math.sin(math.radians(input_angle)))
        max_velocity_possible = math.sqrt((haftreibung * radius_m) / vehicle_mass_kg ) #due to antipetal force

        if max_velocity_possible > input_speed:
            #no oversteering
            delta_m = delta_t * input_speed
            vehicle_x   += math.cos(math.radians(input_angle)+vehicle_yaw)      * delta_m
            vehicle_y   += math.sin(math.radians(input_angle)+vehicle_yaw)      * delta_m
            vehicle_yaw += math.tan(math.radians(input_angle))/vehicle_length_m * delta_m
            vehicle_speed = input_speed
            oversteer.append(0)
        else:
            #oversteering
            #TODO: make more realistic !!!!!!!!
            # let the vehicle break but not turn
            #print(f"Oversteering: req.speed: {input_speed} req.angle: {input_angle} max speed: {max_velocity_possible}", end="")
            factor = input_speed/max_velocity_possible
            input_speed = min(max(input_speed/factor, min_speed), max_speed)
            input_angle /= factor

            #print(f" res. speed: {input_speed} res. angle: {input_angle}")

            delta_m = delta_t * input_speed
            vehicle_x   += math.cos(math.radians(input_angle)+vehicle_yaw)      * delta_m
            vehicle_y   += math.sin(math.radians(input_angle)+vehicle_yaw)      * delta_m
            vehicle_yaw += math.tan(math.radians(input_angle))/vehicle_length_m * delta_m
            vehicle_speed = input_speed
            oversteer.append(1)


        xs.append(vehicle_x)
        ys.append(vehicle_y)
        speeds.append(input_speed)

        print(f"{i}/{total_steps}")
    
    mpc.plotTrajectory(xs, ys, speeds)


    pyplot.plot(error_x, label="Error in X [m]")
    pyplot.plot(error_y, label="Error in Y [m]")
    pyplot.plot(error_yaw, label="Error in Yaw [deg]")
    pyplot.title("Simulated Localization Error over Time")
    pyplot.legend()
    pyplot.show()

    pyplot.plot(computation_times)
    pyplot.title("Time to compute one action [s]")
    pyplot.show()

    pyplot.plot(speeds)
    pyplot.title("Velocity over Time [m/s]")
    pyplot.show()

    """pyplot.plot([(speeds[i] - speeds[i-1])/delta_t for i in range(1, len(speeds))])
    pyplot.plot(oversteer)
    pyplot.show()"""


        

if __name__ == "__main__":
    main()