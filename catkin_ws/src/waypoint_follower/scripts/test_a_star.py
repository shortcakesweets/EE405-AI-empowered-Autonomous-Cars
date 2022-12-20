#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid


map = OccupancyGrid()

def map_callback(msg):
    global map 
    map = msg  

def main():
    rospy.init_node('publish_map')
    rate = rospy.Rate(20)
    pub_map = rospy.Publisher('/repeat_map', OccupancyGrid, queue_size=1)
    sub_map = rospy.Subscriber('/map', OccupancyGrid, callback= map_callback) 
    while not rospy.is_shutdown():
        pub_map.publish(map)
        rate.sleep()

if __name__ == '__main__':
    main()
