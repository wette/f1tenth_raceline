import rclpy

import copy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

from trajectory import Trajectory
from pid_controller import PIDController

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer



import math

TOPIC_DRIVE = "/drive"
TOPIC_LASERSCAN = "/scan"
TOPIC_ODOMETRY = "/ego_racecar/odom"
TOPIC_DEBUG_MARKERARRAY = "/debug/raceline"
TOPIC_DEBUG_MARKER = "/debug/marker"


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        #create publishers
        self.publisher_ackermann        = self.create_publisher(AckermannDriveStamped, TOPIC_DRIVE, 10)
        self.publisher_markerarray_viz  = self.create_publisher(MarkerArray, TOPIC_DEBUG_MARKERARRAY, 10)
        self.publisher_marker_viz       = self.create_publisher(Marker, TOPIC_DEBUG_MARKER, 10)

        #create listeners
        #self.sub_laser = self.create_subscription(LaserScan, TOPIC_LASERSCAN, self.cb_new_laserscan, 10)
        #self.sub_laser  # prevent unused variable warning
        #self.sub_odom  = self.create_subscription(Odometry, TOPIC_ODOMETRY, self.cb_new_odometry, 10)
        #self.sub_odom  # prevent unused variable warning

        #receive transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        #TODO: file path from configuration or command line
        self.raceline.load_trajectory_from_file("/home/wette/wette_racecar_ws/src/raceline/raceline/my_map_raceline.csv")

        self.map_frame_name     = "map"
        self.vehicle_frame_name = "ego_racecar/laser"

        self.lookahead_m = 0.3          #lookahead to find out steering angle
        self.index_on_raceline = 0      #where on the trajectory are we currently?


        self.create_timer(1.0, self.debug_publish_raceline)
        self.create_timer(0.2, self.dodrive)

    def debug_publish_raceline(self):
        minvel = min(self.raceline.velocity_profile)
        maxvel = max(self.raceline.velocity_profile)
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
            m.color.g = (1-(self.raceline.velocity_profile[i]-minvel))*color_scale
            m.color.b = 0.0
            m.color.a = 1.0

            ma.markers.append(m)
        
        self.publisher_markerarray_viz.publish(ma)

    def dodrive(self):
        try:
            self.drive()
        except Exception as e:
            print(e)

    def drive(self):
        #find out where we currently are in the map
        t = self.tf_buffer.lookup_transform(
                                    self.map_frame_name,
                                    self.vehicle_frame_name,
                                    rclpy.time.Time())
        x_vehicle_map = t.transform.translation.x
        y_vehicle_map = t.transform.translation.y

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
                best_idx = i
                found_better_point = True
            else:
                if found_better_point:
                    #last time we found a better point - this time not:
                    # we can end the search as from now on, we will be moving away from the best point
                    break

        #find the required steering angle
        #find next point on trajectory which is self.lookahead awy from current position of vehicle
        best_next_idx = best_idx
        best_diff = 100000
        found_better_point = False
        for i in range(len(self.raceline.x)):
            idx = (best_idx + i) % len(self.raceline.x)
            diff_x = self.raceline.x[idx]-x_vehicle_map
            diff_y = self.raceline.y[idx]-y_vehicle_map
            len_diff = abs(self.lookahead_m - math.sqrt(diff_x*diff_x+diff_y*diff_y))
            if len_diff < best_diff:
                #found better point!
                best_diff = len_diff
                best_next_idx = i
                found_better_point = True
            else:
                if found_better_point:
                    #last time we found a better point - this time not:
                    # we can end the search as from now on, we will be moving away from the best point
                    break

        #found the points in map coordinate system. need to transfer back to vehicle coordinate system
        x = self.raceline.x[best_next_idx]
        y = self.raceline.y[best_next_idx]
        
        t = self.tf_buffer.lookup_transform(self.vehicle_frame_name,
                                    self.map_frame_name,
                                    rclpy.time.Time())
        x += t.transform.translation.x
        y += t.transform.translation.y

        #DEBUG: publish point
        m = Marker()
        m.header.frame_id = self.vehicle_frame_name
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "debug_raceline"
        m.id = 132478
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
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        self.publisher_marker_viz.publish(m)

        #TODO: compute angle to target_point
        

        #find the required speed
        speed = self.raceline.velocity_profile[best_idx]

        #write to VESC
                
        #variable contents for next iteration
        self.index_on_raceline = best_idx



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
