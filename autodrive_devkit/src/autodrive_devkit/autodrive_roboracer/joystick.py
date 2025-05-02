################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.qos import QoSProfile # Ouality of Service (tune communication between nodes)
from std_msgs.msg import Float32, Bool # Float32 and Bool message classes
from sensor_msgs.msg import LaserScan, Joy
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped

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
accel_data = 0
steering_data = 0

def joy_callback(msg):
    """store joy data"""
    # print(" lidar_callback triggered")
    global accel_data, steering_data
    print("run1")
    accel_data = msg.drive.acceleration
    steering_data = msg.drive.steering_angle
    print("run2")
    # print(f"lidar_data: {lidar_data}")
    # print("------------------------------")

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
    global accel_data, steering_data

    # Settings
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # ROS 2 infrastructure
    rclpy.init()
    qos = QoSProfile(depth=1)
    node = rclpy.create_node('joydata')

    sub_speed = node.create_subscription(AckermannDriveStamped, '/input/teleop', joy_callback, qos)

    pub_steering_command = node.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', qos)
    pub_throttle_command = node.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', qos)
    pub_reset_command = node.create_publisher(Bool, '/autodrive/reset_command', qos)
    
    # Initialize
    throttle_msg = Float32()
    steering_msg = Float32()
    reset_msg = Bool()
    throttle = 0.0
    steering = 0.0
    reset_flag = False
    

    try:
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

            # print("run1")
            # print(f"lidar_data type: {type(lidar_data)}")
            # print("sub lidar data:", lidar_data)
            # re-plan trajectory
            # print("run2")
            
            steering = steering_data
            throttle = accel_data

            print(f"throttle, steering: {throttle, steering}")
            print("------------------------------")

            
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