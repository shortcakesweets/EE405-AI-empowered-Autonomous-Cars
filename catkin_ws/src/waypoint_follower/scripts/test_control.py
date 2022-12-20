#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool

def main():
    rospy.init_node('auto_control')
    rate = rospy.Rate(100)
    pub_steer    = rospy.Publisher('/auto_cmd/steer', Int16, queue_size=1)
    pub_throttle = rospy.Publisher('/auto_cmd/throttle', Int16, queue_size=1)
    pub_uno      = rospy.Publisher('/auto_mode', Bool, queue_size=1)
    while not rospy.is_shutdown():
        msg_steer = Int16()
        msg_steer.data = 1700
        pub_steer.publish(msg_steer)

        msg_throttle = Int16()
        msg_throttle.data = 1470
        pub_throttle.publish(msg_throttle)

        msg_uno = Bool()
        msg_uno.data = True
        pub_uno.publish(msg_uno)

        rate.sleep()

if __name__ == '__main__':
    main()