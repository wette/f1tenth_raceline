from sensor_msgs.msg import LaserScan
import math

class LaserscanFilter:
    #set everything to zero which is outside start-end index. Cut all rays to max_lookahead_meters
    @staticmethod
    def clean_up_laserscan(msg: LaserScan, start_idx: int, end_idx: int, max_lookahead_meters = 2.5):
        #set everything to zero that is outside of the search range
        for i in range(0, start_idx):
            msg.ranges[i] = 0

        for i in range(end_idx, len(msg.ranges)):
            msg.ranges[i] = 0

        for i in range(start_idx, end_idx):
            # cut all ray at max_lookahead_meters
            msg.ranges[i] = min(msg.ranges[i], max_lookahead_meters)

    @staticmethod
    def find_largest_gap(start_idx: int, end_idx: int, msg: LaserScan, min_gap_depth_meters = 1.0):
        #
        # find largest consequitive gap
        #
        gap_start_idx = start_idx
        gap_end_idx = start_idx
        gap_largest_idx = start_idx

        #values to return
        largest_start_idx = start_idx
        largest_end_idx = start_idx
        largest_ray_idx = start_idx

        for i in range(start_idx, end_idx):
            if msg.ranges[i] > min_gap_depth_meters:
                gap_end_idx = i

                if msg.ranges[i] > msg.ranges[gap_largest_idx]:
                    gap_largest_idx = i
            else:
                #gap is over. is it the new record holder?
                if(gap_end_idx - gap_start_idx > largest_end_idx - largest_start_idx):
                    largest_start_idx = gap_start_idx
                    largest_end_idx = gap_end_idx
                    largest_ray_idx = gap_largest_idx
                gap_start_idx = i+1
                gap_end_idx = i+1

        #gap is over. is it the new record holder?
        if(gap_end_idx - gap_start_idx > largest_end_idx - largest_start_idx):
            largest_start_idx = gap_start_idx
            largest_end_idx = gap_end_idx
            largest_ray_idx = gap_largest_idx


        return largest_start_idx, largest_end_idx, largest_ray_idx
    
    @staticmethod
    def find_center_of_gap(msg: LaserScan, largest_start_idx: int, largest_end_idx: int, largest_ray_idx: int, threshold = 0.05):
        #
        #   find center of largest gap around the deepest point in the scan (+/-5%)
        #
        max_value = msg.ranges[largest_ray_idx]
        threshold = threshold * max_value

        left_idx  = largest_ray_idx
        right_idx = largest_ray_idx

        for i in range(largest_ray_idx, largest_end_idx, +1):
            if abs(max_value - msg.ranges[i]) < threshold:
                left_idx = i
            else:
                break

        for i in range(largest_ray_idx, largest_start_idx, -1):
            if abs(max_value - msg.ranges[i]) < threshold:
                right_idx = i
            else:
                break

        #from left_idx to right_idx, the depth is nearly the same. choose point in center
        center_of_gap = right_idx + int((left_idx - right_idx)/2.0)

        return center_of_gap

    @staticmethod
    def remove_breaching_rays(msg: LaserScan, threshold_rays=0.3, min_gap_size_m=0.3, min_ray_size_m=0.2):
        #find small gaps in the data where the vehicle could not pass
        start_idx = 0
        end_idx = 0
        in_breach = False
        for i in range(0, len(msg.ranges)-1):
            if not in_breach and msg.ranges[i+1] > min_ray_size_m and msg.ranges[i+1] > (1+threshold_rays) * msg.ranges[i]:
                #found start of a breach
                start_idx = i
                in_breach = True
            if in_breach and msg.ranges[i+1] > min_ray_size_m and msg.ranges[i+1] < (1+threshold_rays) * msg.ranges[i]:
                #found end of a breach
                end_idx = i
                in_breach = False

                #check the size of the breach:
                right_ray_idx = max(start_idx-1, 0)
                left_ray_idx = min(end_idx+1, len(msg.ranges))

                right_ray_length = msg.ranges[right_ray_idx]
                left_ray_length = msg.ranges[left_ray_idx]

                alpha = ((end_idx-1)-(start_idx+1)) * msg.angle_increment

                a2 = right_ray_length*right_ray_length
                b2 = left_ray_length*left_ray_length
                ab = right_ray_length*left_ray_length

                c = math.sqrt( a2+b2 - 2*ab*math.cos(alpha) )

                if c <= min_gap_size_m:
                    #remove gap by interpolating
                    diff = left_ray_length-right_ray_length
                    maxIdx = left_ray_idx-right_ray_idx
                    for j in range(0, maxIdx+1):
                        idx = right_ray_idx+j
                        msg.ranges[idx] = right_ray_length + j/maxIdx * diff

    @staticmethod
    def remove_large_rays(msg: LaserScan, threshold_m=4.0):
        for i in range(0, len(msg.ranges)):
            if msg.ranges[i] > threshold_m:
                msg.ranges[i] = threshold_m
    
    @staticmethod
    def distance_to_wall(msg: LaserScan, left_wall = True):
        #compute steering based on distance to left wall
        #angles are measured counterclockwise
        center_ray_idx = math.ceil(len(msg.ranges) / 2.0)
        theta_deg = 20

        s = 1
        if not left_wall:
            s = -1

        #take measurement 90 degree and 70 degree from x axis (front) of vehicle
        left_90_deg_ray_idx = center_ray_idx + s*math.ceil(90.0 / math.degrees(msg.angle_increment))
        left_70_deg_ray_idx = center_ray_idx + s*math.ceil((90.0 - theta_deg) / math.degrees(msg.angle_increment))
        distance_to_wall_90deg_meters = msg.ranges[left_90_deg_ray_idx]
        distance_to_wall_70deg_meters = msg.ranges[left_70_deg_ray_idx]

        #compute distance to wall (see F1tenth lab assignment 3)
        alpha_rad = math.atan( 
                                (distance_to_wall_70deg_meters * math.cos(math.radians(theta_deg)) - distance_to_wall_90deg_meters) / 
                                (distance_to_wall_70deg_meters * math.sin(math.radians(theta_deg))) 
                             )

        distance_to_wall_meters = distance_to_wall_90deg_meters * math.cos(alpha_rad)

        return distance_to_wall_meters

    @staticmethod
    def smooth_scan(msg: LaserScan, windowlength=3):
        if windowlength % 2 == 0:
            windowlength += 1
        ranges = []

        offset = int((windowlength-1)/2)
        maxidx = len(msg.ranges)-1
        for i in range(0, len(msg.ranges)):
            ranges.append( sum(msg.ranges[max(i-offset, 0):min(i+offset+1, maxidx)])/windowlength )
        msg.ranges = ranges
    
    @staticmethod
    def closeInfinityGaps(msg: LaserScan):
        gapStartIdx = 1
        gapEndIdx = 1
        inGap = False
        for i in range(1, len(msg.ranges)):
            if msg.ranges[i] > msg.range_max:
                if not inGap:
                    inGap = True
                    gapStartIdx = i
                    gapEndIdx = i
                if inGap:
                    gapEndIdx = i
            else:
                if inGap:
                    #gap over!
                    inGap = False
                    #interpolate
                    gapLength = gapEndIdx+1 - (gapStartIdx-1)
                    gapDiff = msg.ranges[gapEndIdx+1] - msg.ranges[gapStartIdx-1]
                    delta = gapDiff / float(gapLength)
                    x = 1
                    for j in range(gapStartIdx, gapEndIdx+1):
                        msg.ranges[j] = msg.ranges[gapStartIdx-1] + delta*x
                        x += 1


