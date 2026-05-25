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
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to the crazyflie at `URI` and runs a square-diagonal
sequence. This script requires some kind of location system and the flow deck.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time
import csv
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
FILENAME = "normalized_error_log.csv"

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def send_position_setpoint(cf, x, y, z, yaw):
    """Send a position setpoint to the Crazyflie."""
    cf.commander.send_position_setpoint(x, y, z, yaw)

def log_position_error(scf, target_x, target_y, target_z, duration=8):
    """Log the normalized position error during movement."""
    log_config = LogConfig(name='Position', period_in_ms=100)
    log_config.add_variable('kalman.stateX', 'float')
    log_config.add_variable('kalman.stateY', 'float')
    log_config.add_variable('kalman.stateZ', 'float')

    errors = []
    start_time = time.time()

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            cf.commander.send_position_setpoint(target_x, target_y, target_z, 0)

            time.sleep(0.2)
            timestamp = log_entry[0]
            data = log_entry[1]

            current_x = data['kalman.stateX']
            current_y = data['kalman.stateY']
            current_z = data['kalman.stateZ']

            x_error = abs(current_x - target_x)
            y_error = abs(current_y - target_y)
            z_error = abs(current_z - target_z)

            target_sum = abs(target_x) + abs(target_y) + abs(target_z) + 1e-6

            normalized_error = (x_error + y_error + z_error) / math.sqrt(target_sum)

            elapsed_time = time.time() - start_time
            errors.append((elapsed_time, normalized_error))
            

            if elapsed_time >= duration:
                break

    return errors

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    all_errors = []

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        # Arm the Crazyflie
        cf.platform.send_arming_request(True)
        time.sleep(1.0)
        print("Errors: ", all_errors)
        # Start position and log errors during movement
        send_position_setpoint(cf, 0.0, 0.0, 0.4, 0)
        all_errors.extend(log_position_error(scf, 0.0, 0.0, 0.4))
        print("Errors: ", all_errors)
        # Move in square-diagonal pattern
        send_position_setpoint(cf, 0.5, 0.5, 0.4, 0)  # Move to top-right corner
        all_errors.extend(log_position_error(scf, 0.5, 0.5, 0.4))
        print("Errors: ", all_errors)
        send_position_setpoint(cf, -0.5, -0.5, 0.4, 0)  # Move to bottom-left corner
        all_errors.extend(log_position_error(scf, -0.5, -0.5, 0.4))
        print("Errors: ", all_errors)
        send_position_setpoint(cf, 0.5, -0.5, 0.4, 0)  # Move to bottom-right corner
        all_errors.extend(log_position_error(scf, 0.5, -0.5, 0.4))
        print("Errors: ", all_errors)
        send_position_setpoint(cf, -0.5, 0.5, 0.4, 0)  # Move to top-left corner
        all_errors.extend(log_position_error(scf, -0.5, 0.5, 0.4))
        print("Errors: ", all_errors)

        # Hover in place before landing
        send_position_setpoint(cf, 0.0, 0.0, 0.4, 0)
        all_errors.extend(log_position_error(scf, 0.0, 0.0, 0.4))

        # Gradually descend
        for y in range(10):
            send_position_setpoint(cf, 0.0, 0.0, (10 - y) / 25.0, 0)
            time.sleep(0.1)

        cf.commander.send_stop_setpoint()
        # Hand control over to the high-level commander to avoid timeout and locking of the Crazyflie
        cf.commander.send_notify_setpoint_stop()

    # Write all errors to a CSV file after flight
    with open(FILENAME, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time", "Normalized_Error"])
        writer.writerows(all_errors)
