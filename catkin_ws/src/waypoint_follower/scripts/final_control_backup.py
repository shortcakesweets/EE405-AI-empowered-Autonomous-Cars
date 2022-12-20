#!/usr/bin/env python
import numpy as np
# import os
import pandas as pd
import math

# import roslib
import rospy
import rospkg

from std_msgs.msg import Int16, String
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud 
from geometry_msgs.msg import PointStamped

import tf
from tf.transformations import euler_from_quaternion

# addtional length
length_path = 0.6
num_points = 5
additions1 = range(num_points)
for i in range(num_points):
    additions1[i] = -0.2 + i*0.1

additions2 = range(7)
for i in range(7):
    additions2[i] = -0.1 + i*0.1

additions3 = range(7)
for i in range(7):
    additions3[i] = 0.1 - i*0.1

total_index = np.array(range(0,len(additions1)))

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

def globalpoint2localpoint(ego_x, ego_y, ego_yaw, x, y):

    R_T = np.array([[math.cos(ego_yaw), math.sin(ego_yaw)], 
                    [-math.sin(ego_yaw), math.cos(ego_yaw)]]) # inverse of rotation matrix from global coordinate to ego's coordinate

    r_wp = np.array([x, y]) # vector global coordinate of a waypoint
    r_ego = np.array([ego_x, ego_y]) # vector position of vehicle
    r_wp_ego = np.dot(R_T, r_wp - r_ego) # vector coordinate of a waypoint w.r.t vehicle's coordinate
    
    output_x = r_wp_ego[0]
    output_y = r_wp_ego[1]

    return output_x, output_y


def localpoint2globalpoint(ego_x, ego_y, ego_yaw, x, y):

    R = np.array([[math.cos(ego_yaw), -math.sin(ego_yaw)], 
                    [math.sin(ego_yaw), math.cos(ego_yaw)]]) # inverse of rotation matrix from global coordinate to ego's coordinate

    r_p = np.array([x, y]) # vector lobal coordinate of a point
    r_ego = np.array([ego_x, ego_y]) # vector position of vehicle
    r_wp_ego = np.dot(R, r_p) + r_ego # vector coordinate of a point w.r.t map's coordinate
    
    output_x = r_wp_ego[0]
    output_y = r_wp_ego[1]

    return output_x, output_y

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


class WaypointFollower():
    def __init__(self):
        # ROS init
        rospy.init_node('wpt_follwer')
        self.rate = rospy.Rate(100.0)

        # Params
        self.target_speed = 0.1 #10/3.6
        self.MAX_STEER    = np.deg2rad(30)
        
        self.wpts_x = None
        self.wpts_y = None
        # Load waypoints
        rospack = rospkg.RosPack()
        WPT_CSV_PATH = rospack.get_path('waypoint_follower') + "/wpt_data/waypoint.csv"
        csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
        self.wpts_x = csv_data.values[:,0]
        self.wpts_y = csv_data.values[:,1]
        print("loaded wpt :", self.wpts_x.shape, self.wpts_y.shape)

        # Params for steering controller
        self.dt = 0.03333
        self.previous_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.prev_error_v = 0
        self.total_error_v = 0
        self.Kp = 0.7
        # self.Kp = 0.1
        self.Ki = 0.01
        # self.Kd = 0.5
        self.Kd = 0.04
        
        self.error_y = 0
        self.prev_error_y = 0
        self.total_error_y = 0
        self.prev_error_yaw = 0
        self.error_yaw = 0

        # vehicle state
        self.ego_x   = 0
        self.ego_y   = 0
        self.ego_yaw = 0
        self.ego_vx  = 0

        self.wpt_look_ahead = 5 # [index]

        # Throttle control
        self.throttle = 1500
        self.stop = False 
        self.have_obstacle = False

        # Obstacle 
        self.obstacle_y = list([])
        self.obstacle_x = list([])

        # Traffic light
        self.traffic_stop = False

        # Pub/Sub
        self.pub_steer    = rospy.Publisher('/auto_cmd/steer', Int16, queue_size=1)
        self.pub_throttle = rospy.Publisher('/auto_cmd/throttle', Int16, queue_size=1)
        self.pub_uno      = rospy.Publisher('/auto_mode', Bool, queue_size=1)
        self.sub_odom     = rospy.Subscriber('/odom', Odometry, self.callback_odom, queue_size = 1)
        self.sub_cv       = rospy.Subscriber('/traffic_color_topic', String, self.callback_cv, queue_size =1)
        self.sub_obstacle = rospy.Subscriber('/obstacle_pointcloud', PointCloud, self.callback_obstacle, queue_size=1)
        
        # visualize la point 
        self._target_point_publisher = rospy.Publisher("/next_target", PointStamped, queue_size=1)
    
    
    def callback_odom(self, msg):
        """
        Subscribe Odometry message
        ref: http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        """
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vx = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

        # get euler from quaternion
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        _, _, self.ego_yaw = euler_from_quaternion(q_list)

        # run_step
        self.previous_time = self.current_time
        self.current_time = rospy.Time.now()
        # self.dt = (self.current_time - self.previous_time).to_sec() 

        if self.wpts_x.shape[0] > 0:
            self.run_step()

    def callback_cv(self, msg):
        traffic = msg.data
        if self.ego_x <= 0.3 and self.ego_x >= -0.7:
            if  traffic == "red" or traffic == "yellow" :
                self.traffic_stop = True
            else:
                self.traffic_stop = False
        else:
            self.traffic_stop = False
    
    def callback_obstacle(self,msg):
        self.obstacle_y = list([])
        self.obstacle_x = list([])
        for point in msg.points:
            self.obstacle_x.append(point.x)
            self.obstacle_y.append(point.y)
        # if len(self.obstacle_x) == 0: self.have_obstacle = False
        # else: self.have_obstacle = True

    def calc_error(self, ego_x, ego_y, ego_yaw, x_list, y_list, obstacle_x_list, obstacle_y_list, wpt_look_ahead=0):

        # 1. Global to Local coordinate
        local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)

        if len(obstacle_x_list) == 0: self.have_obstacle = False
        else: self.have_obstacle = True
        
        # if self.have_obstacle == False: 
            #############
            # Not have obstacle 
            ############
        
        # # 1. Global to Local coordinate
        # local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)

        # 2. Find the nearest waypoint
        _, near_ind = find_nearest_point(ego_x, ego_y, x_list, y_list)

        # 3. Set lookahead waypoint (index of waypoint trajectory)
        lookahead_wpt_ind = near_ind + wpt_look_ahead
        if lookahead_wpt_ind >= len(x_list):
            lookahead_wpt_ind -= len(x_list)

        # 4. postion of lookahead wpt in body frame
        y_la_local = local_y_list[lookahead_wpt_ind]
        x_la_local = local_x_list[lookahead_wpt_ind]
        # self.stop = False

        # 4. Calculate errors
        error_yaw = math.atan2(y_la_local - local_y_list[near_ind],
                            x_la_local - local_x_list[near_ind])
        # error_yaw = math.atan2(y_la_local, x_la_local)
        error_yaw = normalize_angle(error_yaw) # Normalize angle to [-pi, +pi]
        if abs(error_yaw) > 0.15 : self.wpt_look_ahead = 5
        else: self.wpt_look_ahead = 10
        error_y = y_la_local

        
        if self.have_obstacle == False: print("no obstacle")

        else:
            #############
            # Have obstacle 
            ############
            print("have obstacle")
            # 1. Find the near obstacle
            min_dist = float('inf')
            ind = 0
            for index in range(len(obstacle_x_list)):
                dist = math.sqrt(obstacle_x_list[index]**2 + obstacle_y_list[index]**2)
                if min_dist >= dist:
                    ind = index
                    min_dist = dist
            # dist_list = np.array(dist_list)
            # ind = np.argmin(dist_list)
            
            # 2. Position local to global 
            x_ob_local = obstacle_x_list[ind]
            y_ob_local = obstacle_y_list[ind]
  
            # 3. find the nearest wpt with obstacle
            _, near_ind_obs = find_nearest_point(x_ob_local, y_ob_local, local_x_list, local_y_list)

            # 4. Find lookahead point of obstacle in global coordinate
            lookahead_wpt_ind_obs = near_ind_obs
            if lookahead_wpt_ind_obs >= len(x_list):
                lookahead_wpt_ind_obs -= len(x_list)
            
            x_la_global = x_list[lookahead_wpt_ind_obs]
            y_la_global = y_list[lookahead_wpt_ind_obs]
             
            # global total_index
            global additions1
            global additions2
            global additions3

            # additions_y = np.array(additions)
            # additions_x = [0]*len(additions_y)
            # additions_x = np.array(additions_x)

            # additions_x, additions_y = global2local(0, 0, error_yaw, additions_x, additions_y)
            # additions_x[3] = 0
            # additions_y[3] = 0

            additions = additions1

            if ego_y > 1.5 and ego_x <= -1.5 and self.ego_x >= -5.5:
                additions = additions2
            
            if ego_y < 1.5 and ego_x <= -1.5 and self.ego_x >= -5.5:
                additions = additions3

            total_index = np.array(range(0,len(additions)))
            reject_index = []
           
            # 5. check collision of all points
            for index in range(len(additions)):
                # add to global la point:
                if self.ego_x >= 1.9 or self.ego_x <= -6.97:
                    y_check_global = y_la_global 
                    x_check_global = x_la_global + additions[index]
                    # print("add to x")

                else:
                    y_check_global = y_la_global + additions[index]
                    x_check_global = x_la_global 
                    # print("add to y")
                # y_check_global = y_la_global_obs + additions_y[index]
                # x_check_global = x_la_global_obs + additions_x[index]

                
                # change global to local and check collision 
                x_check_local, y_check_local = globalpoint2localpoint(ego_x, ego_y, ego_yaw,x_check_global,y_check_global)

                for k in range(len(obstacle_x_list)):
                    if calc_dist(x_check_local, y_check_local, obstacle_x_list[k], obstacle_y_list[k]) <= 0.2:
                        reject_index.append(index)
                        break
            
            reject_index = np.array(reject_index)        
            
            # 6. Find the list of selected looking ahead points
            select_index_list =  np.setdiff1d(total_index, reject_index)
            
            # 7. Select the best looking ahead point
            if len(select_index_list) == 0:       
                print("Stop")
                # transform back to local
                x_la_local = 0
                y_la_local = 0
                # Calculate errors
                error_yaw = 0
                error_y = 0
                self.stop = True

            else:
                self.stop = False
                i = np.argmin(abs(select_index_list-2))
                selected_index = select_index_list[i]
                if ego_y > 2.5 and ego_x <= -1.5 and self.ego_x >= -5.5:
                    i = np.argmin(abs(select_index_list-4))
                    selected_index = select_index_list[i] 
                if ego_y < 1 and ego_x <= -1.5 and self.ego_x >= -5.5:
                    i = np.argmin(abs(select_index_list-3))
                    selected_index = select_index_list[i]

                print("Select Index: ", selected_index)

                if self.ego_x >= 1.8 or self.ego_x <= -6.97: 
                    y_la_global = y_la_global 
                    x_la_global = x_la_global + additions[selected_index]
                else: 
                    y_la_global = y_la_global + additions[selected_index] 
                    x_la_global = x_la_global 

                # x_la_global = x_la_global_obs + additions_x[selected_index]
                # y_la_global = y_la_global_obs + additions_y[selected_index]
                               
                # transform back to local
                x_la_local, y_la_local = globalpoint2localpoint(ego_x, ego_y, ego_yaw,x_la_global,y_la_global)

                # 4. Calculate errors
                error_yaw = 0.6*math.atan2(y_la_local, x_la_local)
                error_yaw = normalize_angle(error_yaw) # Normalize angle to [-pi, +pi]
                error_y = y_la_local
                # errora_y = y_la_local
            

        # visual taget wpt
        target_point = PointStamped()
        target_point.header.frame_id = "laser"
        target_point.point.x = x_la_local
        target_point.point.y = y_la_local
        target_point.point.z = 0
        self._target_point_publisher.publish(target_point)

        return error_y, error_yaw

    # Controller
    def steer_control(self, error_y, error_yaw, previous_error_y, previous_error_yaw):
        """
        TODO 4.
        Implement a steering controller (PID controller or Pure pursuit or Stanley method).
        You can use not only error_y, error_yaw, but also other input arguments for this controller if you want.
        """
        
        # Stanley method
        # k = 10
        # if self.ego_vx != 0:
        #     steer = 0.1*(error_yaw + math.atan(k*error_y/self.ego_vx))
        # else:
        #     steer = error_yaw
        error_y = 0.7*error_y + 0.3*previous_error_y

        # derivative_error_y = (error_y - self.prev_error_y)/self.dt
        # derivative_error_yaw = (error_yaw - self.prev_error_yaw)/self.dt

        # steer = error_yaw + self.Kp*error_y + self.Kd*derivative_error_y
        # steer = self.Kp*(error_yaw + error_y) + self.Kd*(derivative_error_y + derivative_error_yaw)
        steer = error_yaw + self.Kp*(error_y)
        # Control limit
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)

        return steer

    def speed_control(self):

        if self.traffic_stop: self.throttle = 1530
        else:
            if self.stop: self.throttle = 1530
            else: self.throttle = 1460 #1460
        return self.throttle

    def publish_command(self, steer, throttle):
        """
        Publish command as AckermannDriveStamped
        ref: http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
        """
        msg_steer = Int16()
        # msg_steer.data = int((steer / np.deg2rad(20))*400) + 1500
        # print(steer*180/math.pi)
        msg_steer.data = int(-steer*800 + 1500)
        # print(msg_steer.data)
        msg_steer.data = int(np.clip(msg_steer.data, 1100, 1900))
        self.pub_steer.publish(msg_steer)

        msg_throttle = Int16()
        msg_throttle.data = throttle
        msg_throttle.data = int(np.clip(msg_throttle.data, 1460, 1530)) # 1460
        self.pub_throttle.publish(msg_throttle)

        msg_uno = Bool()
        msg_uno.data = True
        self.pub_uno.publish(msg_uno)
        # self.pub_command.publish(msg)
    
    def run_step(self):
        # Get current state
        # ego_x = self.ego_x
        # ego_y = self.ego_y
        # ego_yaw = self.ego_yaw

        # Lateral error calculation (cross-track error, yaw error)
        self.prev_error_y = self.error_y
        self.prev_error_yaw = self.error_yaw
        self.error_y, self.error_yaw = self.calc_error(self.ego_x, self.ego_y, self.ego_yaw, self.wpts_x, self.wpts_y, self.obstacle_x, self.obstacle_y, wpt_look_ahead=self.wpt_look_ahead)

        # Control
        steer_cmd = self.steer_control(self.error_y, self.error_yaw, self.prev_error_y, self.prev_error_yaw)
        throttle_cmd = self.speed_control()

        # Publish command
        self.publish_command(steer_cmd, throttle_cmd)

def main():
    # Define controller
    wpt_control = WaypointFollower()
    # total_error_y = 0
    # total_error_v = 0
    # n = 0
    while not rospy.is_shutdown():
        pass

    #rospy.loginfo("Commands: (steer=%.3f, accel=%.3f). Errors: (CrossTrackError=%.3f, YawError=%.3f, SpeedError=%.3f)." %(steer_cmd, throttle_cmd, error_y, error_yaw, error_v))
 
        
    # avg_error_y = total_error_y / n
    # avg_error_v = total_error_v / n
    # print("Average cross-track error: %f" % avg_error_y)
    # print("Average velocity error: %f" % avg_error_v)

if __name__ == '__main__':
    main()
