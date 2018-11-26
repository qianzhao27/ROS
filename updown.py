#Author: Qian Zhao
#!/usr/bin/env python

import rospy
import math
from time import time, sleep

from std_msgs.msg import String
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('twist_talker', anonymous=True)
    msg = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        """
        msg.linear.x = 0  #set the linear motion parameters
        msg.linear.y = 0
        msg.linear.z = 0.2
        msg.angular.z = 0
        pub.publish(msg)
        sleep(2)
        rate.sleep()
        """
        for i in range(4):
            msg.linear.x = 0.2  #set the linear motion parameters
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.z = 0
            sleep(2)
            pub.publish(msg)         
            rate.sleep()
        
            msg.linear.x = 0  #set the angular motion parameters
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.z = 45
            sleep(2)
            pub.publish(msg)
            rate.sleep()
        
if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass


