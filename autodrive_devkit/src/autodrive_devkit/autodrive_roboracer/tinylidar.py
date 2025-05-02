import cvxpy as cp
import time
import yaml
import numpy as np
from argparse import Namespace
import warnings
import threading
# from .tiny_lidarnet import TinyLidarNet
from .TinyLidarNet.tiny_lidarnet import TinyLidarNet

print("Hiiii")
import pyglet.gl
print("Bye")


################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.qos import QoSProfile # Ouality of Service (tune communication between nodes)
from std_msgs.msg import Float32, Bool # Float32 and Bool message classes
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

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

CONTROLLER = ['TinyLidarNet']
RACETRACK = 'Spielberg'
VISUALIZE = False

lidar_data = None  # Initialize it with a default value
speed_data = 0

def lidar_callback(msg):
    """store lidar data"""
    global lidar_data
    lidar_data = np.array(msg.ranges, dtype=np.float32)

def speed_callback(msg):
    """store speed data"""
    global speed_data
    speed_data = msg.data

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

def main():
    global lidar_data, speed_data

    # Settings
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # ROS 2 infrastructure
    rclpy.init()
    qos = QoSProfile(depth=1)
    node = rclpy.create_node('tinylidar')

    # sub_lidar = node.create_subscription(LaserScan, '/autodrive/roboracer_1/lidar', lidar_callback, qos)
    sub_lidar = node.create_subscription(LaserScan, '/autodrive/roboracer_1/lidar', lidar_callback, qos_profile_sensor_data)
    sub_speed = node.create_subscription(Float32, '/autodrive/roboracer_1/speed', speed_callback, qos)

    pub_steering_command = node.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', qos)
    pub_throttle_command = node.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', qos)
    pub_reset_command = node.create_publisher(Bool, '/autodrive/reset_command', qos)
    
    # spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    # spin_thread.start()
    
    # Initialize
    throttle_msg = Float32()
    steering_msg = Float32()
    reset_msg = Bool()
    throttle = 0.0
    steering = 0.0
    reset_flag = False
    

    test_id = "benchmark_tiny_il_m"
    
    controller = TinyLidarNet(test_id,2, 0,'/home/autodrive_devkit/src/autodrive_devkit/autodrive_roboracer/TinyLidarNet/f1_tenth_model_small_noquantized.tflite')
    
    try:
        while lidar_data is None:
            print("waiting for lidar data...")
            rclpy.spin_once(node, timeout_sec=0.1)
            
        # main control loop
        while(1):
            rclpy.spin_once(node, timeout_sec=0.1)

            key = get_key(settings)
            if key == 'r' :
                throttle = 0.0
                steering = 0.0
                reset_flag = True

            elif key == '\x03': # CTRL+C
                break
                
            # re-plan trajectory
            steering, desired_speed = controller.plan(lidar_data)

            current_speed = speed_data
            # gain = 1.0
            gain = 0.5
            throttle = (desired_speed - current_speed) * gain
            
            # Generate control messages
            throttle_msg.data = float(throttle)
            steering_msg.data = float(steering)
            reset_msg.data = reset_flag

            # Publish control messages
            pub_throttle_command.publish(throttle_msg)
            pub_steering_command.publish(steering_msg)
            pub_reset_command.publish(reset_msg)

            #Reset the flag
            reset_flag = False
    
    except Exception as error:
        #Print error
        print(error)

    finally:
        #Generate and publis zero commands
        throttle_msg.data = float(0.0)
        steering_msg.data = float(0.0)
        reset_msg.data = False
        pub_throttle_command.publish(throttle_msg)
        pub_steering_command.publish(steering_msg)
        pub_reset_command.publish(reset_msg)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
################################################################################

if __name__ == "__main__":
    main()
