#!/usr/bin/env python

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

KAIROS_MAX_LIN_VEL = 1.50
KAIROS_MAX_ANG_VEL = 3.00
KAIROS_MAX_LIN_ACC = 0.60
KAIROS_MAX_ANG_ACC = 1.50

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your RB-KAIROS!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
  input_vel = max(input_vel, low_bound)
  input_vel = min(input_vel, high_bound)
  return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -KAIROS_MAX_LIN_VEL, KAIROS_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -KAIROS_MAX_ANG_VEL, KAIROS_MAX_ANG_VEL)


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('kairos_teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0
        
    def print_vels():
        nonlocal target_linear_vel
        print(f'currently:\tlinear velocity {target_linear_vel}\t angular velocity {target_angular_vel}')

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w' :
                target_linear_vel = check_linear_limit_velocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                print_vels()
            elif key == 'x' :
                target_linear_vel = check_linear_limit_velocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                print_vels()
            elif key == 'a' :
                target_angular_vel = check_angular_limit_velocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                print_vels()
            elif key == 'd' :
                target_angular_vel = check_angular_limit_velocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                print_vels()
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print_vels()
                status -= 1
            else:
                if (key == '\x03'):
                    break

            # show again controls
            if status == 20 :
                print(msg)
                status = 0

            # send command
            twist = Twist()
            control_linear_vel = make_simple_profile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_angular_vel = make_simple_profile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel
            pub.publish(twist)

    except Exception as ex:
        print(e)
        print(ex)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
  main()