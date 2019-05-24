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
#  MA  02110-1301, USA.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
This example utilizes the SyncCrazyflie and SyncLogger classes.
"""
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


def getData(scf, available):
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=12)
    # lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('pm.vbat', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    with SyncLogger(scf, lg_stab) as logger:
        for log_entry in logger:
            data = log_entry[1]
            y = data.get('stabilizer.yaw')
            vbat = data.get('pm.vbat')
            return(y, vbat)
