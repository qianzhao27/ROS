#Author: Qian Zhao
#!/usr/bin/env python

import rospy
import math
from time import time, sleep

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from math import pow, atan2, sqrt

class Node:
    def __init__(self):
        rospy.init_node('quadrotor_controller1', anonymous=True)
        self.publisher = rospy.Publisher('drone1/cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.pose = Pose()
        self.goal = Point()

    def update_pose(self, data):
        # Callback function which is called when a new message of type Pose is
        # received by the subscriber.
        self.pose.position.x = data.position.x
        self.pose.position.y = data.position.y
        self.pose.position.z = data.position.z
        self.pose.orientation.x = data.orientation.x
        
        self.vel.linear.z = self.z_vel()
        if self.perpendicular() < 0.02:
            self.vel.linear.z = 0
        # self.publisher.publish(self.vel)
        
        if self.euclidean_distance() > 0.1:
            self.vel.linear.x = 0.1
            self.vel.angular.z = self.angular_vel()
        else:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
        self.publisher.publish(self.vel)
        
        print(self.pose.position.x, self.pose.position.y, self.pose.position.z)
        print('yaw:',self.pose.orientation.x, ' steer angle:',self.steering_angle()*180/math.pi)
        print('angular_z:', self.angular_vel())
        print()
        """

        # only set angular.z
        self.vel.linear.x = 0  #set the linear motion parameters
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.z = 45
        self.publisher.publish(self.vel)
        

        
        if self.perpendicular() > float(0.02):
            self.vel.linear.z = self.z_vel()
            self.publisher.publish(self.vel)
        else: 
            self.vel.linear.z = 0
            self.publisher.publish(self.vel)
        """
    
    # calculating vel by following methods
    def euclidean_distance(self):
        return sqrt(pow((self.goal.x - self.pose.position.x), 2) +
                    pow((self.goal.y - self.pose.position.y), 2) )    
    
    def perpendicular(self):
        return sqrt(pow((self.goal.z - self.pose.position.z),2))

    def x_distance(self):
        return sqrt(pow((self.goal.z - self.pose.position.z),2))
   
    def linear_vel(self, constant= 0.2):
        return constant * self.euclidean_distance()

    def z_vel(self,constant= 0.5):
        return constant * (self.goal.z - self.pose.position.z)
 
    def steering_angle(self):
        return atan2(self.goal.y-self.pose.position.y, self.goal.x-self.pose.position.x)
    
    def angular_vel(self):
        return ((self.steering_angle()*180/math.pi) + self.pose.orientation.x)

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
           
            shape = input('Do you want to fly to a goal?')
            n.goal.x = float(input("Set your x goal: "))
            n.goal.y = float(input("Set your y goal: "))
            n.goal.z = float(input("Set your z goal: "))
        subscriber = rospy.Subscriber('drone1/pose', Pose, n.update_pose)

        rospy.spin()
    

if __name__ == '__main__':
     try:
         starter()
     
     except rospy.ROSInterruptException:
         pass


