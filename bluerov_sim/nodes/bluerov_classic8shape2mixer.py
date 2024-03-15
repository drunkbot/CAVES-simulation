#!/usr/bin/env python
import rospy
import yaml
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def groundtruth_callback(msg):
    cmd_vel.linear.z = (-0.9-msg.pose.pose.position.z)*0.5*0.5

    cmd_vel.linear.x = 0.5*0.5*0.6
    cmd_vel.linear.y = 0
    cmd_vel.linear.z = (-0.9-msg.pose.pose.position.z)*0.5*0.5
    cmd_vel.angular.y = 0
    eu_angles=transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    cmd_vel.angular.z = (1.57-eu_angles[2])*0.5*0.5*0.5
    cmd_vel.angular.x = 0
    
    
    pub.publish(cmd_vel)

def joy2cmd():
    global cmd_vel
    # global joy_linear_gain
    # global joy_angular_gain
    global pub

    cmd_vel = Twist()

    # joy_linear_gain = rospy.get_param('~cfg_joy_linear_gain')

    # joy_angular_gain = rospy.get_param('~cfg_joy_angular_gain')

    rospy.Subscriber("/bluerov/ground_truth/state", Odometry, groundtruth_callback)
    # rospy.Subscriber("/bluerov/twist", Twist, joy2cmd_callback)
    pub = rospy.Publisher("twist", Twist, queue_size=2)

    rospy.spin()

if __name__ == '__main__':
    
    with open('/home/kanzhoong/mallard_ws/setpoints.yaml', 'r') as infile:
        setpoints = yaml.safe_load(infile)
    x=[setpoints[i]['x'] for i in range(len(setpoints))]
    y=[setpoints[i]['y'] for i in range(len(setpoints))]
    yaw=[setpoints[i]['yaw'] for i in range(len(setpoints))]

    rospy.init_node('Joy2cmd')
    joy2cmd()
