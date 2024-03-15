#!/usr/bin/env python
import rospy
import threading
# from std_msgs.msg import Float64
# from mavros_msgs.msg import MotorSetpoint
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn


class MixerNode():
    def __init__(self):
        rospy.init_node("mixer")

        self.setpoint_pub = rospy.Publisher("/bluerov/mavros/rc/override",
                                            OverrideRCIn,
                                            queue_size=1)
        self.data_lock = threading.RLock()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.thrust = 0.0
        self.vertical_thrust = 0.0
        self.lateral_thrust = 0.0

        self.twist_sub = rospy.Subscriber("twist", Twist, self.twist_remap)
        self.twist_sub_ry = rospy.Subscriber("twist_rp", Twist, self.twist_rp_remap)

    def twist_remap(self, msg):
        with self.data_lock:
            # self.thrust = 0.71*msg.linear.z
            # self.lateral_thrust = 0.4*msg.linear.y
            # self.vertical_thrust = 0.*msg.linear.x
            # self.pitch = 0.*msg.angular.z 
            # self.roll = 0.*msg.angular.y
            # self.yaw = -0.27*msg.angular.x
            self.thrust = msg.linear.x
            self.lateral_thrust = msg.linear.y
            self.vertical_thrust = msg.linear.z
            # self.pitch =msg.angular.y 
            # self.roll = msg.angular.x
            self.yaw = msg.angular.z

    def twist_rp_remap(self, msg):
        with self.data_lock:

            self.pitch =msg.angular.y 
            self.roll = msg.angular.x
   

    def run(self):
        # Receives joystick messages (subscribed to Joy topic)
        # axis 1 aka left stick vertical
        # axis 0 aka left stick horizontal
        # Joystick data input
        # rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            cmd_vel = OverrideRCIn()
            cmd_vel_y = 1500 + (400 * self.lateral_thrust)
            cmd_vel_x = 1500 + (400 * self.thrust)
            cmd_vel_yaw = 1500 + (400 * self.yaw)
            cmd_vel_z = 1500 + (400 * self.vertical_thrust)
            cmd_vel_roll = 1500 + (400 * self.roll)
            cmd_vel_pitch = 1500 + (400 * self.pitch)
            cmd_vel.channels = [cmd_vel_pitch, cmd_vel_roll, cmd_vel_z, cmd_vel_yaw, cmd_vel_x, cmd_vel_y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.setpoint_pub.publish(cmd_vel)

def main():
    # MixerNode()
    node = MixerNode()
    node.run()


if __name__ == "__main__":
    main()
