#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
This module contains a local planner to perform
low-level waypoint following based on PID controllers.
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import fot
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

import rospkg
import pandas as pd
from nav_msgs.msg import Odometry


from std_msgs.msg import Int16,Bool


left_paths_idx = range(3,5)
right_paths_idx = range(0,2)
straight_paths_idx = range(2,3)



safe_speed=  1.5
max_speed = 30.0
avg_speed = 10.0

 
class MyLocalPlanner(object):
    """
    LocalPlanner implements the basic behavior of following a trajectory of waypoints that is
    generated on-the-fly. The low-level motion of the vehicle is computed by using two PID
    controllers, one is used for the lateral control and the other for the longitudinal
    control (cruise speed).

    When multiple paths are available (intersections) this local planner makes a random choice.
    """

    def __init__(self):
        # ROS init
        rospy.init_node('Planner')
        self.rate = rospy.Rate(100.0)
        
        self.vehicle_yaw = 0
        self.current_speed = 0
        self.current_acc = 0
        self.obstacles = Path()
        self.mapx = None
        self.mapy = None
        self.maps = None
        self.target_speed = safe_speed
        self.ego_x = 0
        self.ego_y = 0
        
        # Parameters for the local planner

        self.c_speed = 0
        self.di = 0
        self.di_d = 0
        self.di_dd = 0
        self.si = 0

        self.x = 0
        self.y = 0   

        # Parameters for the obstacle advoidance 
        
        # Params
        self.MAX_STEER    = np.deg2rad(20)
        
        # Params for speed controller
        self.dt = 1/20
        self.prev_error_v = 0
        self.total_error_v = 0
        self.Kp = 1
        self.Ki = 0.01
        self.Kd = 5 
        self.prev_error_y = 0
        self.total_error_y = 0   

        self._target_point_publisher = rospy.Publisher("/next_target", PointStamped, queue_size=1)

        self.test_pub = rospy.Publisher("/test_hz", String, queue_size=1)

        # Publisher to visualize wrong paths and correct paths
        self.rejected_paths_pub = rospy.Publisher("/rejected_paths", MarkerArray, queue_size=1)
        self.accepted_paths_pub = rospy.Publisher("/accepted_paths", MarkerArray, queue_size=1)
        self.chosen_path_pub = rospy.Publisher("/chosen_paths", Marker, queue_size=1)
        
        # Subscriber to get the current pose of the car
        self.pub_steer    = rospy.Publisher('/auto_cmd/steer', Int16, queue_size=1)
        self.pub_throttle = rospy.Publisher('/auto_cmd/throttle', Int16, queue_size=1)
        self.pub_uno      = rospy.Publisher('/auto_mode', Bool, queue_size=1)
        self.sub_odom     = rospy.Subscriber('/odom', Odometry, self.callback_odom, queue_size=1)
        self.sub_obstacle = rospy.Subscriber('/obstacle', Path, self.callback_obstacle, queue_size=1)

    def callback_obstacle(self,msg):
        self.obstacles = msg

    def vehicle_status(self, vehicle_status):
        quaternion = (
            vehicle_status.orientation.x,
            vehicle_status.orientation.y,
            vehicle_status.orientation.z,
            vehicle_status.orientation.w
        )
        _, _, self.vehicle_yaw = euler_from_quaternion(quaternion)
        
        # current_acc = vehicle_status.acceleration.linear
        # self._current_acc = np.sqrt(current_acc.x ** 2 + current_acc.y ** 2)
        # print("Current Acc: %f" % self._current_acc)
        
    # Calculate distance
    def calc_dist(self, tx, ty, ix, iy):
        return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

    # Normalize angle [-pi, +pi]
    def normalize_angle(self, angle):
        if angle > math.pi:
            norm_angle = angle - 2*math.pi
        elif angle < -math.pi:
            norm_angle = angle + 2*math.pi
        else:
            norm_angle = angle
        return norm_angle
    
    # Global2Local
    def global2local(self, ego_x, ego_y, ego_yaw, x, y):
    
        R_T = np.array([[math.cos(ego_yaw), math.sin(ego_yaw)], 
                        [-math.sin(ego_yaw), math.cos(ego_yaw)]]) # inverse of rotation matrix from global coordinate to ego's coordinate
    
        r_wp = np.array([x, y]) # vector global coordinate of a waypoint
        r_ego = np.array([ego_x, ego_y]) # vector position of vehicle
        r_wp_ego = np.dot(R_T, r_wp - r_ego) # vector coordinate of a waypoint w.r.t vehicle's coordinate
        
        output_x = r_wp_ego[0]
        output_y = r_wp_ego[1]

        return output_x, output_y
    
    # Calculate Error
    def calc_error(self, ego_x, ego_y, ego_yaw, x, y):
        # 1. Global to Local coordinate
        local_x, local_y = self.global2local(ego_x, ego_y, ego_yaw, x, y)

        # 4. Calculate errors
        error_yaw = math.atan2(local_y, local_x)
        error_yaw = self.normalize_angle(error_yaw) # Normalize angle to [-pi, +pi]
        error_y = local_y
        return error_y, error_yaw
    
    # Controller
    def steer_control(self, error_y, error_yaw):
    
        steer = error_yaw + error_y
    
        # Control limit
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)

        return steer
        
    def speed_control(self, error_v):
         
        self.total_error_v += error_v
        throttle = self.Kp * error_v + self.Ki * self.total_error_v + self.Kd * (error_v - self.prev_error_v) / 0.05
        self.prev_error_v = error_v
                
        return throttle

    def publish_command(self, steer, throttle):
        msg_steer = Int16()
        # msg_steer.data = int((steer / np.deg2rad(20))*400) + 1500
        msg_steer.data = int(-steer*800 + 1500)
        msg_steer.data = int(np.clip(msg_steer.data, 1100, 1900))
        self.pub_steer.publish(msg_steer)

        msg_throttle = Int16()
        # msg_throttle.data = int(-throttle*100 + 1500)
        msg_throttle.data = 1470
        msg_throttle.data = int(np.clip(msg_throttle.data, 1470, 1500))
        self.pub_throttle.publish(msg_throttle)

        msg_uno = Bool()
        msg_uno.data = True
        self.pub_uno.publish(msg_uno)
    
    def callback_odom(self, msg):
        """
        Subscribe Odometry message
        ref: http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        """
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.current_speed = safe_speed
        # get euler from quaternion
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        _, _, self.vehicle_yaw = euler_from_quaternion(q_list)
        self.run_step()
    

    def run_step(self):
        """
        Execute one step of local planning which involves running the longitudinal
        and lateral PID controllers to follow the waypoints trajectory.
        """
        
        fot.TARGET_SPEED = self.target_speed

        x = self.ego_x
        y = self.ego_y


        s, d = fot.get_frenet(x, y, self.mapx, self.mapy)
        x, y, yaw_road = fot.get_cartesian(s, d, self.mapx, self.mapy, self.maps)
        yawi = self.vehicle_yaw - yaw_road

        self.si = s
        self.di = d
        self.di_d = self.current_speed * np.sin(yawi)
        self.di_dd = self.current_acc * np.sin(yawi)
        self.di_dd = self.current_acc * np.sin(yawi)
        self.c_speed = self.current_speed * np.cos(yawi)

        fplist = fot.calc_frenet_paths(self.c_speed, self.di, self.di_d, self.di_dd, self.si)
        ind_list, fplist = fot.calc_global_paths(fplist, self.mapx, self.mapy, self.maps, self.obstacles)


        # self.local_goal = self._waypoint_buffer[0].position
        accepted_paths = MarkerArray()
        rejected_paths = MarkerArray()
        # reject_idx = []
        # for i in range(len(fplist)):
        #     reject_idx.append(False)

        
        # Check if the generated frenet paths is collided with obstacle
        for idx, fp in enumerate(fplist):
            msg = Marker()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.get_rostime()
            msg.pose.orientation.w = 1.0
            msg.id = idx
            msg.type = Marker.LINE_STRIP
            msg.color.r, msg.color.g, msg.color.b, msg.color.a = 0.0, 128.0, 0.0, 2.0
            msg.scale.x, msg.scale.y, msg.scale.z = 0.05, 0.05, 0.05
            
            for i in range(len(fp.x)):
                point = Point()
                point.x, point.y, point.z = fp.x[i], fp.y[i], 0
                msg.points.append(point)

            # Add the path to the MarkerArray for visualization
            if not ind_list[idx]:
                msg.color.r, msg.color.g, msg.color.b, msg.color.a = 128.0, 0.0, 0.0, 2.0
                rejected_paths.markers.append(msg)
                msg.color.a = 0.001
                accepted_paths.markers.append(msg)
                msg.color.a = 2.0
            else:
                accepted_paths.markers.append(msg)
                msg.color.a = 0.001
                rejected_paths.markers.append(msg)
                msg.color.a = 2.0

        # Visualize the paths
        self.accepted_paths_pub.publish(accepted_paths)
        self.rejected_paths_pub.publish(rejected_paths)

        # if all(reject_idx[left_paths_idx[0]:left_paths_idx[-1] + 1]) and all(reject_idx[right_paths_idx[0]:right_paths_idx[-1] + 1]) and reject_idx[straight_paths_idx[0]]:

        # Find the optimal path among all generated paths
        min_cost = float("inf")
        opt_traj = fplist[len(fplist) // 2 + 1]

    
        for idx, fp in enumerate(fplist):
            if ind_list[idx]:
                continue
            if min_cost >= fp.c_tot:
                min_cost = fp.c_tot
                opt_traj = fp
        # if opt_traj == None:
        #     print("No solution!")

        chosen_path = Marker()
        chosen_path.header.frame_id = "map"
        chosen_path.header.stamp = rospy.get_rostime()
        chosen_path.pose.orientation.w = 1.0
        chosen_path.id = 100
        chosen_path.type = Marker.LINE_STRIP
        chosen_path.color.r, chosen_path.color.g, chosen_path.color.b, chosen_path.color.a = 255.0, 99.0, 71.0, 2.0
        chosen_path.scale.x, chosen_path.scale.y, chosen_path.scale.z = 0.3, 0.3, 0.3

        for i in range(len(opt_traj.x)):
            point = Point()
            point.x = opt_traj.x[i]
            point.y = opt_traj.y[i]
            point.z = 0
            chosen_path.points.append(point)
            # if i is not 0:
            #     self.local_way.append(point)

        # Visualize chosen path
        self.chosen_path_pub.publish(chosen_path)

        
        #Get the goal point to send to the controller
        # self.x = opt_traj.x[len(opt_traj.x) // 2] 
        # self.y = opt_traj.y[len(opt_traj.y) // 2]
        
        self.x = opt_traj.x[2] 
        self.y = opt_traj.y[2]
        

        # target waypoint
        target_point = PointStamped()
        target_point.header.frame_id = "map"
        target_point.point.x = self.x
        target_point.point.y = self.y
        target_point.point.z = 0
        self._target_point_publisher.publish(target_point)


        # target speed
        # deg_v2pe = abs(math.atan2(opt_traj.y[-1] - current_pose.position.y, opt_traj.x[-1] - current_pose.position.x)*180/math.pi)
        # deg_vehicle = abs(self._vehicle_yaw*180/math.pi)
        # diff_deg_e = abs(deg_v2pe - deg_vehicle)

        
        #if len(self.obstacles) < 1:
            # self.target_speed = MAX_spd
        #    self.target_speed = safe_speed
        #else:
            # self.target_speed = AVG_spd + diff_deg_e * (MIN_spd - AVG_spd)/(60 - 0)
        #    self.target_speed = safe_speed
        self.target_speed = safe_speed

        # move using PID controllers
        # control = self._vehicle_controller.run_step(
        #     self.target_speed, current_speed, current_pose, self.target_route_point)
        
        # Lateral error calculation (cross-track error, yaw error)
        error_y, error_yaw = self.calc_error(self.ego_x, self.ego_y, self.vehicle_yaw, self.x, self.y)

        # Longitudinal error calculation (speed error)
        error_v = self.target_speed - self.current_speed

        # Control
        steer_cmd = self.steer_control(error_y, error_yaw)
        throttle_cmd = self.speed_control(error_v)
        # Publish command
        self.publish_command(steer_cmd, throttle_cmd)


        self.test_pub.publish("hello")

        return
    
def main():
    # Load Waypoint
    rospack = rospkg.RosPack()
    WPT_CSV_PATH = rospack.get_path('waypoint_follower') + "/wpt_data/waypoint.csv"
    csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
    mapx = csv_data.values[:,0]
    mapy = csv_data.values[:,1] - 0.2
    
    #Get maps for each waypoint in the global plan
    maps = np.zeros(len(mapx))
    for i in range(len(mapx) - 1):
        maps[i], _ = fot.get_frenet(mapx[i], mapy[i], mapx, mapy)

        maps[-1] = maps[-2] \
                    + fot.get_dist(mapx[-2], mapy[-2],
                                        mapx[-1], mapy[-1])
                    
    # Define planner
    planner = MyLocalPlanner()
    planner.mapx = mapx
    planner.mapy = mapy
    planner.maps = maps
    while not rospy.is_shutdown():
        pass
        
    
if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass
