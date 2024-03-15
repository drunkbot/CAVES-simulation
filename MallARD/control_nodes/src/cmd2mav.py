#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Twist

# Receives joystick messages (subscribed to Joy topic)
# axis 1 aka left stick vertical
# axis 0 aka left stick horizontal

   
def cmd2mav_callback(cmd_):
    global pub 
    # Receives joystick messages (subscribed to Joy topic)
    # axis 1 aka left stick vertical
    # axis 0 aka left stick horizontal
    # Joystick data input

    cmd_vel = OverrideRCIn()
    cmd_vel_y = 1500 + (400 * cmd_.linear.y)
    cmd_vel_x = 1500 + (400 * cmd_.linear.x)
    cmd_vel_r = 1500 + (400 * cmd_.angular.z)
    cmd_vel.channels = [0, 0, 0, cmd_vel_r, cmd_vel_x, cmd_vel_y, 0, 0]
    pub.publish(cmd_vel)




def cmd2mavros():
    rospy.init_node('Cmd2mav')
    global pub
    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    # rospy.Subscriber("mallard_cmd_vel", Twist, cmd2mav_callback)
    rospy.Subscriber("/mallard/cmd_vel", Twist, cmd2mav_callback)



if __name__ == '__main__':
    cmd2mavros()
    rospy.spin()
