import rclpy

import copy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from trajectory import Trajectory
from pid_controller import PIDController



import math

TOPIC_DRIVE = "/drive"
TOPIC_LASERSCAN = "/scan"
TOPIC_ODOMETRY = "/ego_racecar/odom"
TOPIC_DEBUG_RANGE = "/debug/range"
TOPIC_DEBUG_MARKER = "/debug/raceline"


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        #create publishers
        self.publisher_ackermann    = self.create_publisher(AckermannDriveStamped, TOPIC_DRIVE, 10)
        self.publisher_viz          = self.create_publisher(MarkerArray, TOPIC_DEBUG_MARKER, 1000)
        self.publisher_debug_range  = self.create_publisher(LaserScan, TOPIC_DEBUG_RANGE, 10)

        #create listeners
        #self.sub_laser = self.create_subscription(LaserScan, TOPIC_LASERSCAN, self.cb_new_laserscan, 10)
        #self.sub_laser  # prevent unused variable warning
        #self.sub_odom  = self.create_subscription(Odometry, TOPIC_ODOMETRY, self.cb_new_odometry, 10)
        #self.sub_odom  # prevent unused variable warning

        #status of the vehicle
        self.speed_meters_per_sec       = [0,0,0]   #current velocity of the vehicle
        self.filtered_free_space_ahead  = 0.0       #how much free space is currently in front of the vehicle?
        self.in_emergency_stop          = False
        self.last_steering_angle        = 0         #steering angle of last epoch
        self.time_last_laserscan        = None

        #parameters to filter the steering signal before its given to the VESC
        self.pid = PIDController(kp = 0.3, ki = 0.0, kd = 0.1)

        #dimensions of the vehicle
        self.vehicle_width_meters           = 0.4
        self.vehicle_max_steering_angle_deg = 60

        #raceline
        #TODO: Trajectroy constructor w/o arguments...
        self.raceline = Trajectory(x=[0, 1], y=[0, 1], haftreibung=0, vehicle_width_m=0, vehicle_acceleration_mss=0, vehicle_deceleration_mss=0, resolution=0)
        self.raceline.load_trajectory_from_file("/home/wette/sim_ws/src/hsbi/f1tenth_raceline/raceline/my_map_raceline.csv")

        self.debug_publish_raceline()

    def debug_publish_raceline(self):
        minvel = min(self.raceline.velocity_profile)
        maxvel = max(self.raceline.velocity_profile)
        color_scale = 1.0/(maxvel-minvel)
        ma = MarkerArray()
        for i in range(len(self.raceline.x)):
            m = Marker()
            m.header.frame_id = "map"
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
            m.color.g = (1-(self.raceline.velocity_profile[i]-minvel))*color_scale
            m.color.b = 0.0
            m.color.a = 1.0

            ma.markers.append(m)
        
        self.publisher_viz.publish(ma)

    #destructor
    def __del__(self):
        pass
        #self.stop_vehicle()


def main(args=None):
    rclpy.init(args=args)

    pure_pursuit = PurePursuit()

    rclpy.spin(pure_pursuit)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pure_pursuit.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
