
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__)))

import math
import time
import random

import rclpy
from rclpy.executors import MultiThreadedExecutor

import copy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from pid_controller import PIDController

from laserscan_filter import LaserscanFilter

from telemetry_monitor_interfaces.msg import Telemetry

import aesthetic_control_interfaces.srv as ae_srv
import aesthetic_control_interfaces.msg as ae_msg

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import ExtrapolationException
import tf2_geometry_msgs #import required to compute transform!

from raceline_msgs.msg import Trajectory


from trajectory import VehicleDescription
from mpcController import MPCController

import yaml


TOPIC_RACELINE = "/raceline/trajectory"
TOPIC_LASERSCAN = "/scan"
TOPIC_ODOMETRY = "/odom"
TOPIC_DEBUG_MARKERARRAY = "/debug/raceline"
TOPIC_DEBUG_MARKER = "/debug/marker"
TOPIC_DEBUG_TELEMETRY = "/debug/telemetry"
TOPIC_LOCALIZATION_COVARIANCE = "/amcl_pose"


class MPCNodeCollisionResolution(Node):

    def __init__(self):
        super().__init__('mpc_controller_collision_resolution')

        #read yaml config file
        with open(os.path.dirname(__file__) + '/../../../../share/raceline/mpc_configuration.yaml', 'r') as f:
            config = yaml.load(f, Loader=yaml.SafeLoader)

        #configuration
        vehicle_update_rate = config["vehicle_update_rate"]    # updates are sent to the vehicle with this rate
        collision_resolution_rate = config["collision_resolution_rate"]    # how often to check for collisions
        lookahead_m = config["lookahead_m"]            # lookahead to prevent collisions
        points_per_meter= config["points_per_meter"]          # number of points per meter to follow

        #create publishers
        self.publisher_raceline        = self.create_publisher(Trajectory, TOPIC_RACELINE, 10)

        #receive transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #create listeners
        self.sub_laser = self.create_subscription(LaserScan, TOPIC_LASERSCAN, self.cb_new_laserscan, 1)
        self.sub_laser  # prevent unused variable warning

        self.sub_odom     = self.create_subscription(Odometry, TOPIC_ODOMETRY, self.callback_on_odom, 1)
        self.sub_odom   # prevent unused variable warning


        self.map_frame_name     = config["map_frame_name"]
        self.vehicle_frame_name = config["vehicle_frame_name"]
        
        self.vehicle_update_rate = vehicle_update_rate

        self.create_timer(1.0/collision_resolution_rate, self.collision_resolution)

        self.vehicle_current_velocity = 0.0
        self.last_steering_angle_rad = 0.0
        self.steering_angle_history_rad = []

        #create MPC object
        #vehicle description
        vehicle_length_m            = config["vehicle_length_m"] 
        haftreibung                 = config["haftreibung"]     #kg force to move standing vehicle in lateral direction
        vehicle_mass_kg             = config["vehicle_mass_kg"]
        vehicle_width_m             = config["vehicle_width_m"]     #half width is minimum distance to any wall at any time
        vehicle_acceleration_mss    = config["vehicle_acceleration_mss"]     #vehicle acceleration in meters/sec/sec
        vehicle_deceleration_mss    = config["vehicle_deceleration_mss"]     #vehicle deceleration in meters/sec/sec
        vehicle_max_steering_angle = config["vehicle_max_steering_angle"]
        vd = VehicleDescription(haftreibung, 
                                vehicle_width_m,  
                                vehicle_mass_kg, 
                                vehicle_acceleration_mss, 
                                vehicle_deceleration_mss, 
                                vehicle_length_m, 
                                -vehicle_max_steering_angle, 
                                vehicle_max_steering_angle)

        self.mpc = MPCController(mapconfigfile=config["mapconfigfile"],
                                raceline=      config["raceline"],
                                vehicle_description=vd, 
                                lookahead_m=lookahead_m,
                                points_per_meter=points_per_meter)
        
        self.raceline = None


    def callback_on_odom(self, msg: Odometry):
        self.vehicle_current_velocity = float( math.sqrt( msg.twist.twist.linear.x **2 + msg.twist.twist.linear.y **2) )

    def cb_new_laserscan(self, msg: LaserScan):
        x_vehicle_map, y_vehicle_map, yaw_vehicle_map = self.get_vehicle_position(time=None)#msg.header.stamp)
        self.mpc.callback_new_laser(msg, x_vehicle_map, y_vehicle_map, yaw_vehicle_map)


    def get_vehicle_position(self, time=None):
        
        if time is None:
            time = rclpy.time.Time() #now
        try:
        
            t = self.tf_buffer.lookup_transform(
                                            self.map_frame_name,
                                            self.vehicle_frame_name,
                                            time)
        except ExtrapolationException as e:
            print(e, flush=True)
            t = self.tf_buffer.lookup_transform(
                                            self.map_frame_name,
                                            self.vehicle_frame_name,
                                            rclpy.time.Time())
        
        x_vehicle_map = t.transform.translation.x
        y_vehicle_map = t.transform.translation.y
        
        q = t.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_vehicle_map = math.atan2(siny_cosp, cosy_cosp)

        return x_vehicle_map, y_vehicle_map, -yaw_vehicle_map


    def collision_resolution(self):
        #find out where we currently are in the map
        x_vehicle_map, y_vehicle_map, yaw_vehicle_map = None, None, None
        try:
            x_vehicle_map, y_vehicle_map, yaw_vehicle_map = self.get_vehicle_position()

        except:
            print(f"No valid transform from {self.map_frame_name} to {self.vehicle_frame_name}. Doin' nothing.", flush=True)
            return
        
        #update MPC
        #print("collision Resolution...", flush=True)
        success = self.mpc.trajectory_collision_resolution( x_vehicle_map, 
                                                            y_vehicle_map, 
                                                            yaw_vehicle_map, 
                                                            self.vehicle_current_velocity,
                                                            delta_t=1.0/self.vehicle_update_rate)
        
        #print("successful: ", success, flush=True)
        if success:
            #publish new trajectory
            msg = Trajectory()
            msg.header.stamp = rclpy.time.Time().to_msg()
            msg.frame_id = "map_pixels"
            
            trajectory =  self.mpc.get_fine_trajectory_pixels()
            msg.num_points = len(trajectory.x)
            msg.xs = trajectory.x
            msg.ys = trajectory.y
            msg.curvature = trajectory.curvature

            if trajectory.velocity_profile is not None:
                msg.velocities = trajectory.velocity_profile
            
            self.publisher_raceline.publish(msg)
            
        else:
            print("Could not find a collision resolution :(", flush=True)
            #TODO: emergency stop



def main(args=None):
    rclpy.init(args=args)

    mpc = MPCNodeCollisionResolution()

    
    rclpy.spin(mpc)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mpc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
