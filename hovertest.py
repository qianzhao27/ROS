#Author: Qian Zhao
#!/usr/bin/env python

import rospy
import math
from time import time, sleep

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import pow, atan2, sqrt

class Node:
    def __init__(self):
        rospy.init_node('quadrotor_controller', anonymous=True)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.pose = Point()

    def update_pose(self, data):
        # Callback function which is called when a new message of type Pose is
        # received by the subscriber.
        self.pose.x = data.x
        self.pose.y = data.y
        self.pose.z = data.z

        print(self.pose.x, self.pose.y, self.pose.z)
        self.vel.linear.x = 0  #set the linear motion parameters
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.z = 45
        self.publisher.publish(self.vel)

def starter():
    # rospy.init_node('quadrotor_controller', anonymous=True)
    # publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # vel = Twist()
    # rate = rospy.Rate(10)
    n = Node()
    shape = 'none'
    # pose = Point()
    while not rospy.is_shutdown():
            
        while shape == 'none': 
            n.vel.linear.x = 0  #set the linear motion parameters
            n.vel.linear.y = 0
            n.vel.linear.z = 0
            n.vel.angular.z = 0
          
            n.publisher.publish(n.vel)         
            n.rate.sleep()
           
            shape = input('Do you want to fly in a sqaure?')
        subscriber = rospy.Subscriber('/position', Point, n.update_pose)

        rospy.spin()
    

if __name__ == '__main__':
     try:
         starter()
     
     except rospy.ROSInterruptException:
         pass


