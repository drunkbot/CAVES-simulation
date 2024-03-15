#!/usr/bin/env python
import rospy
# from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def joy2cmd_callback(msg):
    # Receive joystick messages (subscribed to joy topic)
    # cmd_vel.linear.x = joy_linear_gain*joyData.axes[1]  # left stick Vertical
    # cmd_vel.linear.y = joy_linear_gain*joyData.axes[0]  # left stick Horizontal
    # cmd_vel.angular.z = joy_angular_gain*joyData.axes[3] # right stick Horizontal
    cmd_vel.linear.z = msg.linear.z
    cmd_vel.linear.y = msg.linear.z
    cmd_vel.linear.x = -msg.linear.y
    cmd_vel.angular.y = msg.angular.y 
    cmd_vel.angular.z = -msg.angular.x
    cmd_vel.angular.x = msg.angular.x



    # cmd_vel.linear.z = joyData.buttons[0]  # X
    # cmd_vel.angular.x = joyData.buttons[5] # R1
    # cmd_vel.angular.y = joyData.buttons[4] # L1
    
    pub.publish(cmd_vel)

def joy2cmd():
    global cmd_vel
    # global joy_linear_gain
    # global joy_angular_gain
    global pub

    cmd_vel = Twist()

    # joy_linear_gain = rospy.get_param('~cfg_joy_linear_gain')

    # joy_angular_gain = rospy.get_param('~cfg_joy_angular_gain')

    #rospy.Subscriber("joy", Joy, joy2cmd_callback)
    rospy.Subscriber("/mallard/twist", Twist, joy2cmd_callback)
    pub = rospy.Publisher("/mallard/cmd_vel", Twist, queue_size=2)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Joy2cmd')
    joy2cmd()
