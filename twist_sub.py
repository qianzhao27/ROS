#!/usr/bin/env python
import rospy

from std_msgs.msg import Int32
from random import randint
from geometry_msgs.msg import Twist

def random_callback(msg):
    # rospy.loginfo("I heard %s", msg.linear)
    print('linear:')
    print(msg.linear)
    print('angular:')
    print(msg.angular)

if __name__=='__main__':
    rospy.init_node('twist_subscriber')
   
    sub=rospy.Subscriber('cmd_vel', Twist, random_callback)
    rospy.spin()
