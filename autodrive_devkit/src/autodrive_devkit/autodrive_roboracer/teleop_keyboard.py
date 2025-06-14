#!/usr/bin/env python3

################################################################################

# Copyright (c) 2025, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.qos import QoSProfile # Ouality of Service (tune communication between nodes)
from std_msgs.msg import Float32, Bool # Float32 and Bool message classes

# Python mudule imports
import os # Miscellaneous operating system interfaces
import select # Waiting for I/O completion
import sys # System-specific parameters and functions
if os.name == 'nt':
    import msvcrt # Useful routines from the MS VC++ runtime
else:
    import termios # POSIX style tty control
    import tty # Terminal control functions

################################################################################

# Parameters
DRIVE_LIMIT = 1.0
STEER_LIMIT = 1.0
DRIVE_STEP_SIZE = 0.2
STEER_STEP_SIZE = 0.2

# Information
info = """
-----------------------------------------
AutoDRIVE - RoboRacer Teleoperation Panel
-----------------------------------------

               Q   W   E
               A   S   D
                   X
                   R

W/S : Increase/decrease drive command
D/A : Increase/decrease steer command
Q   : Zero steer
E   : Emergency brake
X   : Force stop and zero steer
R   : Soft-reset the simulator
Press CTRL+C to quit

NOTE: Press keys within this terminal
-----------------------------------------
"""

# Error
error = """
ERROR: Communication failed!
"""

# Get keyboard key
def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Constrain control commands
def constrain(input, low_bound, high_bound):
    if input < low_bound:
        input = low_bound
    elif input > high_bound:
        input = high_bound
    else:
        input = input
    return input

# Constrain steer (lateral) command
def bound_steer(steer_cmd):
    steer_cmd = constrain(steer_cmd, -STEER_LIMIT, STEER_LIMIT)
    return steer_cmd

# Constrain drive (longitudinal) command
def bound_drive(drive_cmd):
    drive_cmd = constrain(drive_cmd, -DRIVE_LIMIT, DRIVE_LIMIT)
    return drive_cmd

################################################################################

def main():
    # Settings
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # ROS 2 infrastructure
    rclpy.init()
    qos = QoSProfile(depth=1)
    node = rclpy.create_node('teleop_keyboard')
    pub_steering_command = node.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', qos)
    pub_throttle_command = node.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', qos)
    pub_reset_command = node.create_publisher(Bool, '/autodrive/reset_command', qos)
    
    node.get_logger().info("Node started successfully!")
    print("Node started successfully!")

    # Initialize
    throttle_msg = Float32()
    steering_msg = Float32()
    reset_msg = Bool()
    throttle = 0.0
    steering = 0.0
    reset_flag = False

    try:
        # Print information
        print(info)

        # Generate control commands
        while(1):
            key = get_key(settings)
            if key == 'w' :
                throttle = bound_drive(throttle + DRIVE_STEP_SIZE)
            elif key == 's' :
                throttle = bound_drive(throttle - DRIVE_STEP_SIZE)
            elif key == 'a' :
                steering = bound_steer(steering + STEER_STEP_SIZE)
            elif key == 'd' :
                steering = bound_steer(steering - STEER_STEP_SIZE)
            elif key == 'q' :
                steering = 0.0
            elif key == 'e' :
                throttle = 0.0
            elif key == 'x' :
                throttle = 0.0
                steering = 0.0
            elif key == 'r' :
                throttle = 0.0
                steering = 0.0
                reset_flag = True
            else:
                if (key == '\x03'): # CTRL+C
                    break
            
            # Generate control messages
            throttle_msg.data = float(throttle)
            steering_msg.data = float(steering)
            reset_msg.data = reset_flag

            print(f"Publishing steering: {steering_msg.data}")
            node.get_logger().info(f"Publishing steering: {steering_msg.data}")

            # Publish control messages
            pub_throttle_command.publish(throttle_msg)
            pub_steering_command.publish(steering_msg)
            pub_reset_command.publish(reset_msg)
            
            # Reset the flag
            reset_flag = False

    except Exception as error:
        # Print error
        print(error)

    finally:
        # Generate and publish zero commands
        throttle_msg.data = float(0.0)
        steering_msg.data = float(0.0)
        reset_msg.data = False
        pub_throttle_command.publish(throttle_msg)
        pub_steering_command.publish(steering_msg)
        pub_reset_command.publish(reset_msg)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

################################################################################

if __name__ == '__main__':
    main()