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


logging.basicConfig(level=logging.ERROR)
URI = 'radio://0/70/2M'

class Node:
    
    def __init__(self):
        
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.va = 0
        
        self.vel = Twist()
        self.vel.linear.x = self.vx
        self.vel.linear.y = self.vy
        self.vel.linear.z = self.vz
        self.vel.angular.z = self.va
        
        # self.vel_pub = rospy.Publisher('cf1/cmd_vel', Twist, queue_size=10)

        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.crazyflie= SyncCrazyflie(URI, cf = Crazyflie(rw_cache='./cache'))
        self.commander = MotionCommander(self.crazyflie)
        self.cf = Crazyflie()
        self.crazyflie.open_link()
        self.commander.take_off()
    def shut_down(self):
        try:
            self.pitch_log.stop()
        finally:
            self.crazyflie.close_link()
    
    def run_node(self):
        
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0.2
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        self.commander._set_vel_setpoint(0,0,0.2,0)
        time.sleep(2)

def cmdVelChanged(msg):
    
    self.vx = msg.linear.x
    self.vy = msg.linear.y
    self.vz = msg.linear.z
    self.va = msg.angular.z
    self.commander._set_vel_setpoint(vx,vy,vz,va)

def run():
    
    rospy.init_node('crazyflie')
    node = Node()
    cmdVel_subscribe = rospy.Subscriber('cmd_vel', Twist, cmdVelChanged)
    while not rospy.is_shutdown():
        #node.run_node()
        rospy.sleep(0.1)
    node.shut_down()

if __name__ == '__main__':
    run()
