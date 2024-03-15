#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn

def joylight_callback(joyData):

    cmd_light_ = OverrideRCIn()
    light = joyData.axes[5] 

    cmd_light = 1500 - (light * 400)
    cmd_light_.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,cmd_light, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    pub.publish(cmd_light_)




def joylight():
    
    global pub
    
    rospy.Subscriber("joy", Joy, joylight_callback)
    pub = rospy.Publisher("/bluerov/mavros/rc/override", OverrideRCIn, queue_size=20)



if __name__ == '__main__':
    rospy.init_node('Joylight')
    joylight()
    rospy.spin()