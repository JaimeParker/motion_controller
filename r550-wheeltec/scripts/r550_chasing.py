"""
Description:
1. use UWB message to acquire velocity and position of agent1 and agent2
2. send data to RL model function, get v_desired and a_desired
3. transfer the data to ros message, type=Twist
4. publish ros message
"""

import rospy
from geometry_msgs.msg import Twist

# init ros node
rospy.init_node('turtlebot_teleop')

# define ros publisher
vel_pub = rospy.Publisher('~/cmd_vel', Twist, queue_size=10)

# # compose uwb data
# def uwb_solver():
#     return pos_vel1, pos_vel2


# # RL trained model output
# def rl_strategy(pos_vel_chaser, pos_vel_runner):
#     return strategy


# publish ros topic
def publish2chaser(strategy):
    # strategy to Twist message
    twist = Twist()
    twist.linear.x = strategy.velocity
    twist.angular.z = strategy.angular
    vel_pub.publish(twist)


if __name__ == "__main__":
    while 1:
        # uwb_solver
        # rl_strategy
        publish2chaser()
