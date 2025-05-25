#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def odom_to_pose(msg):
    pose = PoseStamped()
    pose.header = msg.header
    pose.pose = msg.pose.pose
    pub.publish(pose)

rospy.init_node('odom_to_pose_converter')
sub = rospy.Subscriber('/vins_fusion/camera_pose', Odometry, odom_to_pose)
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
rospy.spin()