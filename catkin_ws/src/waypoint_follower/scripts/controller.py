#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
import math

import roslib
import rospy
import rospkg

from std_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

import tf
from tf.transformations import euler_from_quaternion

# Calculate distance
def calc_dist(tx, ty, ix, iy):
    return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

# Normalize angle [-pi, +pi]
def normalize_angle(angle):
    if angle > math.pi:
        norm_angle = angle - 2*math.pi
    elif angle < -math.pi:
        norm_angle = angle + 2*math.pi
    else:
        norm_angle = angle
    return norm_angle

# Global2Local
def global2local(ego_x, ego_y, ego_yaw, x_list, y_list):
    """
    TODO 1.
    Transform from global to local coordinate w.r.t. ego vehicle's current pose (x, y, yaw).
        - x_list, y_list               : global coordinate trajectory.
        - ego_x, ego_y, ego_yaw        : ego vehicle's current pose.
        - output_x_list, output_y_list : transformed local coordinate trajectory.
    """
    
    output_x_list = []
    output_y_list = []
    R_T = np.array([[math.cos(ego_yaw), math.sin(ego_yaw)], 
                    [-math.sin(ego_yaw), math.cos(ego_yaw)]]) # inverse of rotation matrix from global coordinate to ego's coordinate
    
    for i in range(len(x_list)):
        r_wp = np.array([x_list[i], y_list[i]]) # vector global coordinate of a waypoint
        r_ego = np.array([ego_x, ego_y]) # vector position of vehicle
        r_wp_ego = np.dot(R_T, r_wp - r_ego) # vector coordinate of a waypoint w.r.t vehicle's coordinate
        output_x_list.append(r_wp_ego[0])
        output_y_list.append(r_wp_ego[1])
        
    output_x_list = np.array(output_x_list)
    output_y_list = np.array(output_y_list)

    return output_x_list, output_y_list

# Find nearest point
def find_nearest_point(ego_x, ego_y, x_list, y_list):
    """
    TODO 2.
    Find the nearest distance(near_dist) and its index(near_ind) w.r.t. current position (ego_x,y) and given trajectory (x_list,y_list).
        - Use 'calc_dist' function to calculate distance.
        - Use np.argmin for finding the index whose value is minimum.
    """

    near_ind = -1
    near_dist = 0
    dist_list = []
    for i in range(len(x_list)):
        dist_list.append(calc_dist(ego_x, ego_y, x_list[i], y_list[i]))
    dist_list = np.array(dist_list)
    
    near_ind = np.argmin(dist_list)
    near_dist = dist_list[near_ind]

    return near_dist, near_ind

# Calculate Error
def calc_error(ego_x, ego_y, ego_yaw, x_list, y_list, wpt_look_ahead=0):
    """
    TODO 3.
    1. Transform from global to local coordinate trajectory.
    2. Find the nearest waypoint.
    3. Set lookahead waypoint.
        - (hint) use the index of the nearest waypoint (near_ind) from 'find_nearest_point'.
        - (hint) consider that the next index of the terminal waypoint is 0, which is not terminal_index + 1.
    4. Calculate errors
        - error_yaw (yaw error)
            : (hint) use math.atan2 for calculating angle btw lookahead and next waypoints.
            : (hint) consider that the next index of the terminal waypoint is 0, which is not terminal_index + 1.
        - error_y (crosstrack error)
            : (hint) y value of the lookahead point in local coordinate waypoint trajectory.
    """
    # 1. Global to Local coordinate
    local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)

    # 2. Find the nearest waypoint
    _, near_ind = find_nearest_point(ego_x, ego_y, x_list, y_list)

    # 3. Set lookahead waypoint (index of waypoint trajectory)
    lookahead_wpt_ind = near_ind + wpt_look_ahead
    if lookahead_wpt_ind >= len(x_list):
        lookahead_wpt_ind -= len(x_list)

    # 4. Calculate errors
    error_yaw = math.atan2(local_y_list[lookahead_wpt_ind] - local_y_list[near_ind],
                           local_x_list[lookahead_wpt_ind] - local_x_list[near_ind])
    error_yaw = normalize_angle(error_yaw) # Normalize angle to [-pi, +pi]
    error_y = local_y_list[near_ind]
    

    return error_y, error_yaw

class WaypointFollower():
    def __init__(self):
        # ROS init
        rospy.init_node('wpt_follwer')
        self.rate = rospy.Rate(100.0)

        # Params
        self.target_speed = 0.1 #10/3.6
        self.MAX_STEER    = np.deg2rad(17.75)
        
        # Params for speed controller
        self.dt = 0.01
        self.prev_error_v = 0
        self.total_error_v = 0
        self.Kp = 1
        self.Ki = 0.01
        self.Kd = 5
        
        self.prev_error_y = 0
        self.total_error_y = 0

        # vehicle state
        self.ego_x   = 0
        self.ego_y   = 0
        self.ego_yaw = 0
        self.ego_vx  = 0

        self.wpt_look_ahead = 6   # [index]

        # Pub/Sub
        self.pub_command = rospy.Publisher('/control', AckermannDriveStamped, queue_size=5)
        self.sub_odom    = rospy.Subscriber('/simulation/bodyOdom', Odometry, self.callback_odom)

    def callback_odom(self, msg):
        """
        Subscribe Odometry message
        ref: http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        """
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vx = msg.twist.twist.linear.x
        # get euler from quaternion
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        _, _, self.ego_yaw = euler_from_quaternion(q_list)

    # Controller
    def steer_control(self, error_y, error_yaw):
        """
        TODO 4.
        Implement a steering controller (PID controller or Pure pursuit or Stanley method).
        You can use not only error_y, error_yaw, but also other input arguments for this controller if you want.
        """
        
        # Stanley method
        k = 10
        if self.ego_vx != 0:
            steer = 0.1*(error_yaw + math.atan(k*error_y/self.ego_vx))
        else:
            steer = error_yaw
            

        # Control limit
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)

        return steer
    
        # Pure pursuit method
        # k = 0.28
        # if self.ego_vx != 0:
        #     steer = math.atan((2*0.257*math.sin(error_yaw))/(k*self.ego_vx))
        # else:
        #     steer = 0
        
        # # PID method
        # self.total_error_y += error_y
        # steer = 0.09 * error_y + 0 * self.total_error_y + 1.40625 * (error_y - self.prev_error_y) / self.dt
        # self.prev_error_y = error_y

    def speed_control(self, error_v):
        """
        TODO 5.
        Implement a speed controller (PID controller).
        You can use not only error_v, but also other input arguments for this controller if you want.
        """
         
        self.total_error_v += error_v
        throttle = self.Kp * error_v + self.Ki * self.total_error_v + self.Kd * (error_v - self.prev_error_v) / 0.01
        self.prev_error_v = error_v
                
        return throttle

    def publish_command(self, steer, accel):
        """
        Publish command as AckermannDriveStamped
        ref: http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
        """
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steer / np.deg2rad(17.75)
        msg.drive.acceleration = accel
        self.pub_command.publish(msg)

def main():
    # Load Waypoint
    rospack = rospkg.RosPack()
    WPT_CSV_PATH = rospack.get_path('waypoint_follower') + "/wpt_data/waypoint.csv"
    csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
    wpts_x = csv_data.values[:,0]
    wpts_y = csv_data.values[:,1]

    print("loaded wpt :", wpts_x.shape, wpts_y.shape)

    # Define controller
    wpt_control = WaypointFollower()
    total_error_y = 0
    total_error_v = 0
    n = 0

    while not rospy.is_shutdown():
        # Get current state
        ego_x = wpt_control.ego_x
        ego_y = wpt_control.ego_y
        ego_yaw = wpt_control.ego_yaw
        ego_vx = wpt_control.ego_vx

        # Lateral error calculation (cross-track error, yaw error)
        error_y, error_yaw = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, wpt_look_ahead=wpt_control.wpt_look_ahead)

        # Longitudinal error calculation (speed error)
        error_v = wpt_control.target_speed - ego_vx

        # Control
        steer_cmd = wpt_control.steer_control(error_y, error_yaw)
        throttle_cmd = wpt_control.speed_control(error_v)
        
        # Calculate total cross-track error and velocity error
        total_error_y += abs(error_y)
        total_error_v += abs(error_v)
        n += 1

        # Publish command
        wpt_control.publish_command(steer_cmd, throttle_cmd)

        rospy.loginfo("Commands: (steer=%.3f, accel=%.3f). Errors: (CrossTrackError=%.3f, YawError=%.3f, SpeedError=%.3f)." %(steer_cmd, throttle_cmd, error_y, error_yaw, error_v))
        wpt_control.rate.sleep()
        
    avg_error_y = total_error_y / n
    avg_error_v = total_error_v / n
    print("Average cross-track error: %f" % avg_error_y)
    print("Average velocity error: %f" % avg_error_v)

if __name__ == '__main__':
    main()