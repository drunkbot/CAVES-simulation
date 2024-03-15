#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
    
# build inverted matrix
# A is control allocation matrix
# 0.2075 and 0.2375 are moment arms measured from the centre of Mallard, i.e. half the distance between thrusters

A = np.mat("1 1 0 0; 0 0 -1 -1; 0.2075 -0.2075 0.2375 -0.2375")
# Calculate the Moore--Penrose Pseudoinverse
pseudoinv = np.linalg.pinv(A)


def cmd2thr_callback(cmd):

    global pub
    cmd_ = np.matrix([[cmd.linear.x], [cmd.linear.y], [cmd.angular.z]])
    thr_out = pseudoinv * cmd_

    thr1 = thr_out[0,:]
    thr2 = thr_out[1,:]
    thr3 = thr_out[2,:]
    thr4 = thr_out[3,:] 

    cmd_thr = JointState()
    cmd_thr.header.stamp = rospy.Time.now()
    cmd_thr.name = ['x_thr_left', 'x_thr_right', 'y_thr_left', 'y_thr_right']
    cmd_thr.effort = [thr1, thr2, thr3, thr4]
    
    pub.publish(cmd_thr)


def cmd2thr():
    global pub

    rospy.Subscriber("/mallard/cmd_vel", Twist, cmd2thr_callback)
    pub = rospy.Publisher("/mallard/thruster_command", JointState, queue_size=2)
    
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('Cmd2thr')
    cmd2thr()
    # rospy.spin()
