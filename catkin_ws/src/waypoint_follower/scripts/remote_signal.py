#!/usr/bin/env python

# import roslib
import rospy

from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('/remote_signal', Bool, queue_size=1)
    rospy.init_node('talker_rs', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(True)
        #rospy.loginfo('hi')
        rate.sleep()

    
if __name__ == '__main__':
    talker()

