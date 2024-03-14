import rclpy

import copy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

from trajectory import Trajectory, VehicleDescription
from pid_controller import PIDController

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs #import required to compute transform!



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

        #receive transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #parameters to filter the steering signal before its given to the VESC
        self.pid = PIDController(kp = 1.0, ki = 0.0, kd = 0.0)

        #dimensions of the vehicle
        self.vehicle_width_meters           = 0.4


        #raceline
        #TODO: Trajectroy constructor w/o arguments...
        vd = VehicleDescription(haftreibung=0.0, vehicle_width_m=0.0, vehicle_acceleration_mss=0.0, vehicle_deceleration_mss=0.0)
        self.raceline = Trajectory(x=[0, 1, 2], y=[0, 1, 2], vehicle_description=vd, resolution=0)
        #TODO: file path from configuration or command line
        self.raceline.load_trajectory_from_file("/home/wette/wette_racecar_ws/src/raceline/raceline/my_map_raceline.csv")

        self.map_frame_name     = "map"
        self.vehicle_frame_name = "ego_racecar/base_link"

        self.max_raceline_speed = max(self.raceline.velocity_profile)

        self.lookahead_m = 0.6           #lookahead to find out steering angle
        self.speed_factor = 0.7          #how much of the speed do we want to apply?
        self.speed_min = 0.1             #minimum speed
        self.speed_max = 9.0             #maximum speed
        self.index_on_raceline = -1      #where on the trajectory are we currently?

        self.last_steering_angle_rad = 0.0


        self.create_timer(1.0, self.debug_publish_raceline)
        self.create_timer(1.0/20.0, self.dodrive)

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
            m.color.g = 1.0-m.color.r
            m.color.b = 0.0
            m.color.a = 1.0

            ma.markers.append(m)
        
        self.publisher_markerarray_viz.publish(ma)

    def dodrive(self):
        try:
            self.drive()
        except Exception as e:
            print(e)

    def transformPoint(self, x:float, y:float, from_frame: str, target_frame:str) -> (float, float):
        point_in_map = PoseStamped()
        point_in_map.pose.position.x = x
        point_in_map.pose.position.y = y
        point_in_map.header.frame_id = from_frame
        point_in_map.header.stamp = rclpy.time.Time().to_msg() #get the newest transform
        point_in_vehicle = self.tf_buffer.transform(point_in_map, target_frame)
        
        x = point_in_vehicle.pose.position.x
        y = point_in_vehicle.pose.position.y

        return x,y

    def dynamic_lookahead(self, raceline_index):
        factor = self.raceline.velocity_profile[raceline_index] / self.max_raceline_speed
        return( max(self.lookahead_m, 5.0*factor*factor*self.lookahead_m) )

    def drive(self):
        #find out where we currently are in the map
        t = self.tf_buffer.lookup_transform(
                                    self.map_frame_name,
                                    self.vehicle_frame_name,
                                    rclpy.time.Time())
        x_vehicle_map = t.transform.translation.x
        y_vehicle_map = t.transform.translation.y

        #check if we have lost tracking of the raceline
        if self.index_on_raceline > 0:
            diff_x = self.raceline.x[self.index_on_raceline]-x_vehicle_map
            diff_y = self.raceline.y[self.index_on_raceline]-y_vehicle_map
            len_diff = math.sqrt(diff_x*diff_x+diff_y*diff_y)

            if len_diff > 2.0:
                self.index_on_raceline = -1

        #print(f"x_vehicle_map: {x_vehicle_map}, y_vehicle_map: {y_vehicle_map}")

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
                    break
        
        current_distance_from_raceline = best_diff

        #print(f"closest point: {best_idx}")
                
        #lookahead distance should be adaptable to current conditions
        #in curves, it should be smaller, on straights it should be larger
        lookahead = self.dynamic_lookahead(best_idx)

        #find the required steering angle
        #find next point on trajectory which is self.lookahead awy from current position of vehicle
        best_next_idx = best_idx
        best_diff = 100000
        found_better_point = False
        for i in range(len(self.raceline.x)):
            idx = (best_idx + i) % len(self.raceline.x)
            diff_x = self.raceline.x[idx]-x_vehicle_map
            diff_y = self.raceline.y[idx]-y_vehicle_map
            len_diff = abs(lookahead - math.sqrt(diff_x*diff_x+diff_y*diff_y))
            if len_diff < best_diff:
                #found better point!
                best_diff = len_diff
                best_next_idx = idx
                found_better_point = True
            else:
                if found_better_point:
                    #last time we found a better point - this time not:
                    # we can end the search as from now on, we will be moving away from the best point
                    break

        #found the points in map coordinate system. need to transfer back to vehicle coordinate system
        x = self.raceline.x[best_idx]
        y = self.raceline.y[best_idx]

        x, y = self.transformPoint(x, y, self.map_frame_name, self.vehicle_frame_name)

        #DEBUG: publish closest point
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

        x = self.raceline.x[best_next_idx]
        y = self.raceline.y[best_next_idx]

        #transform fram map to vehicle coordinate system        
        x, y = self.transformPoint(x, y, self.map_frame_name, self.vehicle_frame_name)


        #publish target point
        m = Marker()
        m.header.frame_id = self.vehicle_frame_name
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "debug_raceline"
        m.id = 132479
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
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        self.publisher_marker_viz.publish(m)

        #compute angle to target_point (x,y) (already in vehicle frame)
        b = x
        c = math.sqrt(x*x + y*y)
        alpha = 0.0
        if c != 0.0:
            alpha = math.acos(b/c) #in rad

        #check if we need to steer left or right.
        if y < 0:
            alpha *= -1.0


        #print(f"point: {x}, {y} - b = {b}, c = {c}, alpha = {alpha}", flush=True)
        
        #use PID filter for steering signal
        alpha = self.pid.update(alpha)

        #find the required speed
        speed = self.raceline.velocity_profile[best_idx] * self.speed_factor

        #make sure speed is between fixed min and max values
        speed = max(self.speed_min, min(speed, self.speed_max))

        if current_distance_from_raceline > 0.2:
            speed *= 0.3

        #write to VESC
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = self.vehicle_frame_name
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = float(alpha)
        drive_msg.drive.speed = float(speed)
        self.publisher_ackermann.publish(drive_msg)
                
        #variable contents for next iteration
        self.index_on_raceline = best_idx
        self.last_steering_angle_rad = alpha


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
