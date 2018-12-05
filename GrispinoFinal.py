# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA
"""
Connects to the crazyflie at `URI` and runs a circle
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig

x = []
y = []

#The address for the drone (change according to the drone you are using)
URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def Logger():
    
    timestamp = log_entry[0]
    data = log_entry[1]
    logconf_name = log_entry[2]
    
    #y.append(data['stabilizer.roll'])      The variable just wont append for whatever reason
    #x.append(data['stabilizer.pitch'])     Error type: AttributeError: 'int' object has no attribute 'append'
    
    #Prints pitch, yaw, and roll
    print('[%d][%s]: %s' % (timestamp, logconf_name, data))



if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

#Syncs with Crazyflie
    with SyncCrazyflie(URI) as scf:
        cf = scf.cf
       
       #Logging Data
        lg_stab = LogConfig(name='gyro', period_in_ms=10)
        lg_stab.add_variable('gyro.x', 'float')
        lg_stab.add_variable('gyro.y', 'float')
        lg_stab.add_variable('gyro.z', 'float')

        with SyncLogger(scf, lg_stab) as logger:

            for log_entry in logger:
                Logger()
                break
