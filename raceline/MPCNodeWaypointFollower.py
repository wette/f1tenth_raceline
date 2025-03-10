
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

from raceline_msgs.msg import Trajectory

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import ExtrapolationException
import tf2_geometry_msgs #import required to compute transform!


from trajectory import VehicleDescription
from mpcController import MPCController

import yaml

TOPIC_DRIVE = "/drive"
TOPIC_LASERSCAN = "/scan"
TOPIC_ODOMETRY = "/odom"
TOPIC_RACELINE = "/raceline/trajectory"
TOPIC_DEBUG_MARKERARRAY = "/debug/raceline"
TOPIC_DEBUG_MARKER = "/debug/marker"
TOPIC_DEBUG_TELEMETRY = "/debug/telemetry"
TOPIC_LOCALIZATION_COVARIANCE = "/amcl_pose"


class MPCNodeWaypointFollower(Node):

    def __init__(self):
        super().__init__('mpc_controller_waypoint_follower')

        #read yaml config file
        with open(os.path.dirname(__file__) + '/../../../../share/raceline/mpc_configuration.yaml', 'r') as f:
            config = yaml.load(f, Loader=yaml.SafeLoader)

        #configuration
        vehicle_update_rate = config["vehicle_update_rate"]    # updates are sent to the vehicle with this rate
        collision_resolution_rate = config["collision_resolution_rate"]    # how often to check for collisions
        lookahead_m = config["lookahead_m"]            # lookahead to prevent collisions
        points_per_meter= config["points_per_meter"]          # number of points per meter to follow

        #create publishers
        self.publisher_ackermann        = self.create_publisher(AckermannDriveStamped, TOPIC_DRIVE, 10)
        self.publisher_markerarray_viz  = self.create_publisher(MarkerArray, TOPIC_DEBUG_MARKERARRAY, 10)
        self.publisher_marker_viz       = self.create_publisher(Marker, TOPIC_DEBUG_MARKER, 10)
        self.publisher_telemetry        = self.create_publisher(Telemetry, TOPIC_DEBUG_TELEMETRY, 10)

        #receive transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #create listeners
        self.sub_odom     = self.create_subscription(Odometry, TOPIC_ODOMETRY, self.callback_on_odom, 1)
        self.sub_odom   # prevent unused variable warning

        self.sub_raceline = self.create_subscription(Trajectory, TOPIC_RACELINE, self.cb_new_raceline, 1)
        self.sub_raceline  # prevent unused variable warning

        self.vehicle_update_rate = vehicle_update_rate #updates are sent to the vehicle with 30Hz

        self.map_frame_name     = config["map_frame_name"]
        self.vehicle_frame_name = config["vehicle_frame_name"]

        #self.create_timer(1.0/5.0, self.debug_publish_raceline)
        self.create_timer(1.0/self.vehicle_update_rate, self.dodrive)

        self.create_timer(5.0, self.debug_print_rate)
        self.numExecutions = 0

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
        vehicle_max_steering_angle  = config["vehicle_max_steering_angle"]
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

    def debug_print_rate(self):
        print(f"Control loop execution rate: {self.numExecutions/5.0}Hz", flush=True)
        self.numExecutions = 0


    def publish_point(self, x, y, id, r, g, b, namespace):
        m = Marker()
        m.header.frame_id = self.map_frame_name
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = namespace
        m.id = id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 0.0
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        self.publisher_marker_viz.publish(m)


    def callback_on_odom(self, msg: Odometry):
        self.vehicle_current_velocity = float( math.sqrt( msg.twist.twist.linear.x **2 + msg.twist.twist.linear.y **2) )

    #received a new trajectory to follow
    def cb_new_raceline(self, msg: Trajectory):
        self.mpc.set_fine_trajectory(msg.xs, msg.ys, msg.curvature, msg.velocities)


    def dodrive(self):
        self.drive()

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

    def drive(self):
        #find out where we currently are in the map
        x_vehicle_map, y_vehicle_map, yaw_vehicle_map = None, None, None
        try:
            x_vehicle_map, y_vehicle_map, yaw_vehicle_map = self.get_vehicle_position()

        except:
            print(f"No valid transform from {self.map_frame_name} to {self.vehicle_frame_name}. Doin' nothing.", flush=True)
            drive_msg = AckermannDriveStamped()
            drive_msg.header.frame_id = self.vehicle_frame_name
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.drive.steering_angle = float(self.last_steering_angle_rad)
            drive_msg.drive.speed = float(0.0)
            self.publisher_ackermann.publish(drive_msg)
            return
        
        #update MPC
        input_speed, input_angle = self.mpc.compute_next_command(   x_vehicle_map, 
                                                                    y_vehicle_map, 
                                                                    yaw_vehicle_map, 
                                                                    self.vehicle_current_velocity,
                                                                    delta_t=1.0/self.vehicle_update_rate)
        

        if input_speed is None and input_angle is None:
            print("No Trajectory received, yet. Doin' nothing.", flush=True)
            return
        
        
        input_angle *= -1.0
        #input_speed = 0.0

        #restrict steering angle under high speeds
        if self.vehicle_current_velocity > 3.0:
            input_angle = max(min(input_angle, 4.0), -4.0)

        #write to VESC
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = self.vehicle_frame_name
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = float(math.radians(input_angle))
        drive_msg.drive.speed = float(input_speed)
        self.publisher_ackermann.publish(drive_msg)

        self.last_steering_angle_rad = float(math.radians(input_angle))
        self.steering_angle_history_rad.append(self.last_steering_angle_rad)
        self.steering_angle_history_rad = self.steering_angle_history_rad[-10:]
        self.numExecutions += 1

    def debug_publish_raceline(self):

        self.raceline = self.mpc.get_fine_trajectory_meters(numEntries=10)
        if self.raceline is None:
            print("No raceline found to publish.", flush=True)
            return
        
        self.raceline.compute_velocity_profile()
        
        minvel = 1.0 #min(self.raceline.velocity_profile)
        maxvel = 10.0 #max(self.raceline.velocity_profile)
        color_scale = 1.0
        if (maxvel-minvel) != 0.0:
            color_scale = 1.0/(maxvel-minvel)
        ma = MarkerArray()
        for i in range(len(self.raceline.x)):
            m = Marker()
            m.header.frame_id = self.map_frame_name
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "debug_raceline"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = self.raceline.x[i]
            m.pose.position.y = self.raceline.y[i]
            m.pose.position.z = 0.0
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 0.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.r = (self.raceline.velocity_profile[i]-minvel)*color_scale
            m.color.g = 1.0-m.color.r
            m.color.b = 0.0
            m.color.a = 1.0

            ma.markers.append(m)
        
        self.publisher_markerarray_viz.publish(ma)



def main(args=None):
    rclpy.init(args=args)

    time.sleep(2.0)

    mpc = MPCNodeWaypointFollower()

    
    rclpy.spin(mpc)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mpc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
