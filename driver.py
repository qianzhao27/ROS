###!/usr/bin/env python3
import rospy
import cflib
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander


logging.basicConfig(level=logging.ERROR)
URI = 'radio://0/80/2M'

class Node:
    
    def __init__(self):
        
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.va = 0
        
        self.vel = Twist()
        self.pose = Point()

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
        self.commander.take_off() ###
    
    def logconfig(self):
        #Logging Data
        lg_position = LogConfig(name='stateEstimate', period_in_ms=10)
        lg_position.add_variable('stateEstimate.x', 'float')
        lg_position.add_variable('stateEstimate.y', 'float')
        lg_position.add_variable('stateEstimate.z', 'float')
        return lg_position

    def sync(self,position_pub):
        # logger = SyncLogger(self.crazyflie, self.logconfig())
        with SyncLogger(self.crazyflie, self.logconfig()) as logger:
            for log_entry in logger:
                timestamp = log_entry[0]
                data = log_entry[1]
                logconf_name = log_entry[2]

                x = round(data['stateEstimate.x'],2)
                y = round(data['stateEstimate.y'],2)
                z = round(data['stateEstimate.z'],2)
                
                self.pose.x = x
                self.pose.y = y
                self.pose.z = z
                
                position_pub.publish(self.pose)
                # print('x:',x,' y:',y, ' z:', z)   ###

                # print('[%d][%s]: %s' % (timestamp, logconf_name, data))
                # print(type(data))
                # print(data)
                # if time.time() > endTime:
                #    break
    
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

    def cmdVel(self,msg):
    
        #print(msg.linear)
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.va = msg.angular.z
        self.commander._set_vel_setpoint(self.vx,self.vy,self.vz,self.va) ###
        #rospy.loginfo(msg.linear.x)
        print(self.vx, self.vy, self.vz, self.va)

def run():
    
    rospy.init_node('crazyflie')
    node = Node()
    cmdVel_subscribe = rospy.Subscriber('cmd_vel', Twist, node.cmdVel)
    position_pub = rospy.Publisher('position', Point, queue_size=10)   
    while not rospy.is_shutdown():
        #node.run_node()
        node.sync(position_pub)
        rospy.sleep(0.1)
    node.shut_down()

if __name__ == '__main__':
    run()

