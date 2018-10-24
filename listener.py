###!/usr/bin/env python3
import rospy
import cflib
from geometry_msgs.msg import Twist

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

vel = Twist()

"""def recTwist(data):
    vel.linear.x = data.linear.x
    vel.linear.y = data.linear.y
    vel.linear.z = data.linear.z
    vel.angular.z = data.angular.z
"""
def pub():
    rospy.init_node('cf', anonymous=True)

    # Publisher which will publish to the topic 'cmd_vel'.
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0.2
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        call(0,0,0.2,0)
            
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.z = 180
        velocity_publisher.publish(vel_msg)
        call(0,0,0,180)

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = -0.2
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        call(0,0,-0.2,0)


def call(x,y,z,yawrate):
    
    URI = 'radio://0/80/1M'
    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)
    if __name__ == '__main__':
        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers(enable_debug_driver=False)

        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf
            with MotionCommander(scf) as mc:
                time.sleep(0)
            
                mc._set_vel_setpoint(x,y,z,yawrate)
                time.sleep(1)
                mc.stop() 

if __name__ == '__main__':
    pub()


