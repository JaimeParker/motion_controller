#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty


# define default speed
v_linear_x = 0
v_linear_y = 0
v_angular = 0


# get keyboard input
# from Willow Garage, Inc.
def getKey():
    tty.setraw(sys.stdin.fileno())
    r_list, _, _ = select.select([sys.stdin], [], [], 0.1)
    if r_list:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# show info of speed and angular
def vel_info(speed, turn, angular):
    return "currently:\tlinear x %s\tlinear y %s\t angular z %s" % (speed, turn, angular)


# main
if __name__=="__main__":
    # get keyboard info
    settings = termios.tcgetattr(sys.stdin)

    # init ros node
    rospy.init_node('turtlebot_teleop')

    # topic name: node + cmd_vel
    publisher = rospy.Publisher('~cmd_vel', Twist, queue_size=10)

    try:
        print("keyboard control connected...")
        print(vel_info(v_linear_x, v_linear_y, v_angular))

        while(1):
            key = getKey()

            if key == 'w':
                v_linear_x += 0.1
            elif key == 's':
                v_linear_x -= 0.1
            elif key == 'a':
                v_linear_y += 0.1
            elif key == 'd':
                v_linear_y -= 0.1
            elif key == 'o':
                v_angular += 0.1
            elif key == 'p':
                v_angular -= 0.1
            elif key == 'k':
                v_linear_x = 0
                v_linear_y = 0
                v_angular = 0

            print(vel_info(v_linear_x, v_linear_y, v_angular))

            twist = Twist()
            twist.linear.x = v_linear_x
            twist.linear.y = v_linear_y
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = v_angular

            publisher.publish(twist)

    except Exception as exception:
        print(exception)

    # set all parameters to 0 when the program is finished
    finally:
        twist = Twist()
        twist.linear.x = 0;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        publisher.publish(twist)
        print("finish program")

    #程序结束前设置终端相关属性
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
