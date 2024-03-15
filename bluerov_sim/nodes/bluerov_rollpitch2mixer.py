#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf import transformations




def sub_callback(msg):

    eu_angles=transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

    cmd_vel.linear.z = 0.5*0
    cmd_vel.linear.y = 0.5*0
    cmd_vel.linear.x = 0.5*0

    cmd_vel.angular.y = -0.8*eu_angles[1]
    cmd_vel.angular.z = 0.5*0
    cmd_vel.angular.x = 0.8*eu_angles[0]
    
    pub.publish(cmd_vel)

def joy2cmd():
    global cmd_vel
    # global joy_linear_gain
    # global joy_angular_gain
    global pub
    cmd_vel = Twist()

    # joy_linear_gain = rospy.get_param('~cfg_joy_linear_gain')

    # joy_angular_gain = rospy.get_param('~cfg_joy_angular_gain')

    rospy.Subscriber("/bluerov/mavros/imu/data", Imu, sub_callback)

    # rospy.Subscriber("/bluerov/twist", Twist, joy2cmd_callback)
    pub = rospy.Publisher("twist_rp", Twist, queue_size=2)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rp2cmd')
    joy2cmd()
