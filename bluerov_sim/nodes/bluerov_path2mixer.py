#!/usr/bin/env python
import rospy
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def groundtruth_callback(msg):
    # Receive joystick messages (subscribed to joy topic)
    if msg.pose.pose.position.y<1 and msg.pose.pose.position.x<0.2:
        cmd_vel.linear.x = 0.5*0.5*0.6
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = (-0.9-msg.pose.pose.position.z)*0.5*0.5
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = 0
        cmd_vel.angular.x = 0
    else :
        if msg.pose.pose.position.x<1 and msg.pose.pose.position.y>1.1:
            cmd_vel.linear.x = 0.5*0.5*0.6
            cmd_vel.linear.y = 0
            cmd_vel.linear.z = (-0.9-msg.pose.pose.position.z)*0.5*0.5
            cmd_vel.angular.y = 0
            eu_angles=transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            cmd_vel.angular.z = -eu_angles[2]*0.5*0.5*0.5
            cmd_vel.angular.x = 0
        else :
            if msg.pose.pose.position.y>-1 and msg.pose.pose.position.x<1.8:
                cmd_vel.linear.x = 0.5*0.5*0.6
                cmd_vel.linear.y = 0
                cmd_vel.linear.z = (-0.9-msg.pose.pose.position.z)*0.5*0.5
                cmd_vel.angular.y = 0
                eu_angles=transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                cmd_vel.angular.z = (-1.57-eu_angles[2])*0.5*0.5*0.5
                cmd_vel.angular.x = 0
            else:
                if msg.pose.pose.position.x<2.6 and msg.pose.pose.position.y<-1.1:
                    cmd_vel.linear.x = 0.5*0.5*0.6
                    cmd_vel.linear.y = 0
                    cmd_vel.linear.z = (-0.9-msg.pose.pose.position.z)*0.5*0.5
                    cmd_vel.angular.y = 0
                    eu_angles=transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                    cmd_vel.angular.z = -eu_angles[2]*0.5*0.5*0.5
                    cmd_vel.angular.x = 0
                else :
                    if msg.pose.pose.position.y<1 and  1.5<msg.pose.pose.position.x>2.2 :
                        cmd_vel.linear.x = 0.5*0.5*0.6
                        cmd_vel.linear.y = 0
                        cmd_vel.linear.z = (-0.9-msg.pose.pose.position.z)*0.5*0.5
                        cmd_vel.angular.y = 0
                        eu_angles=transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                        cmd_vel.angular.z = (1.57-eu_angles[2])*0.5*0.5*0.5
                        cmd_vel.angular.x = 0
                    else:
                        cmd_vel.linear.z = (-1.2-msg.pose.pose.position.z)*0.5*0.5
                        cmd_vel.linear.y = 0
                        cmd_vel.linear.x = 0
                        cmd_vel.angular.y = 0
                        cmd_vel.angular.z = 0
                        cmd_vel.angular.x = 0

                                
    # cmd_vel.linear.z = 0.5*0.5
    # cmd_vel.linear.y = 0.5*0.5
    # cmd_vel.linear.x = 0.5*0.5
    # cmd_vel.angular.y = 0.5*0.5
    # cmd_vel.angular.z = 0.2*0.5
    # cmd_vel.angular.x = 0.5*0.5
    
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
    rospy.init_node('Joy2cmd')
    joy2cmd()
