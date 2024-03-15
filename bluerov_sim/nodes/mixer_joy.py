#!/usr/bin/env python
import rospy
import threading
# from std_msgs.msg import Float64
from mavros_msgs.msg import MotorSetpoint
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped



class MixerNode():
    def __init__(self):
        rospy.init_node("mixer")

        self.setpoint_pub = rospy.Publisher("mavros/setpoint_motor/setpoint",
                                            MotorSetpoint,
                                            queue_size=1)

        self.data_lock = threading.RLock()

        self.thruster = self.init_mixing()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.thrust = 0.0
        self.vertical_thrust = 0.0
        self.lateral_thrust = 0.0

        # self.roll_sub = rospy.Subscriber("roll",
        #                                  Float64,
        #                                  self.on_roll,
        #                                  queue_size=1)
        # self.pitch_sub = rospy.Subscriber("pitch",
        #                                   Float64,
        #                                   self.on_pitch,
        #                                   queue_size=1)
        # self.yaw_sub = rospy.Subscriber("yaw",
        #                                 Float64,
        #                                 self.on_yaw,
        #                                 queue_size=1)
        # self.thrust_sub = rospy.Subscriber("thrust",
        #                                    Float64,
        #                                    self.on_thrust,
        #                                    queue_size=1)
        # self.vertical_thrust_sub = rospy.Subscriber("vertical_thrust",
        #                                             Float64,
        #                                             self.on_vertical_thrust,
        #                                             queue_size=1)
        # self.lateral_thrust_sub = rospy.Subscriber("lateral_thrust", Float64,
        #                                            self.on_lateral_thrust)
        self.twist_sub = rospy.Subscriber("twist", Twist, self.twist_remap)
        rospy.Subscriber("/bluerov/ground_truth/state", Odometry, self.groundtruth_callback)

    def twist_remap(self, msg):
        with self.data_lock:
            self.thrust = msg.linear.x
            self.lateral_thrust = msg.linear.y
            # self.vertical_thrust = msg.linear.z
            self.pitch = msg.angular.x
            self.roll = msg.angular.y
            self.yaw = msg.angular.z

    def groundtruth_callback(self, msg):
        with self.data_lock:
            self.vertical_thrust = (-0.9-msg.pose.pose.position.z)*0.5*0.5
            # # Initialize the tf broadcaster
            # br = tf2_ros.TransformBroadcaster()
            # transform = TransformStamped()

            # # Set the header information on the transform message
            # transform.header.stamp = msg.header.stamp
            # transform.header.frame_id = msg.header.frame_id
            # transform.child_frame_id = msg.child_frame_id

            # # Set the transform to the pose of the robot
            # transform.transform.translation.x = msg.pose.pose.position.x
            # transform.transform.translation.y = msg.pose.pose.position.y
            # transform.transform.translation.z = msg.pose.pose.position.z
            # transform.transform.rotation = msg.pose.pose.orientation

            # # Broadcast the transform
            # br.sendTransform(transform)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            msg = self.mix()
            self.setpoint_pub.publish(msg)
            rate.sleep()

    def mix(self):
        msg = MotorSetpoint()
        msg.header.stamp = rospy.Time.now()
        with self.data_lock:
            for i in range(8):
                msg.setpoint[i] = (
                    self.roll * self.thruster[i]["roll"] +
                    self.pitch * self.thruster[i]["pitch"] +
                    self.yaw * self.thruster[i]["yaw"] +
                    self.thrust * self.thruster[i]["thrust"] +
                    self.vertical_thrust * self.thruster[i]["vertical_thrust"] +
                    self.lateral_thrust * self.thruster[i]["lateral_thrust"])
        return msg

    def init_mixing(self):
        thruster = [None] * 8
        # roll, pitch, yaw, thrust, lateral thrust, vertical thrust
        thruster[0] = dict(roll=0.0,
                           pitch=0.0,
                           yaw=1.0,
                           thrust=1.0,
                           lateral_thrust=1.0,
                           vertical_thrust=0.0)
        thruster[1] = dict(roll=0.0,
                           pitch=0.0,
                           yaw=-1.0,
                           thrust=1.0,
                           lateral_thrust=-1.0,
                           vertical_thrust=0.0)
        thruster[2] = dict(roll=0.0,
                           pitch=0.0,
                           yaw=1.0,
                           thrust=1.0,
                           lateral_thrust=-1.0,
                           vertical_thrust=0.0)
        thruster[3] = dict(roll=0.0,
                           pitch=0.0,
                           yaw=-1.0,
                           thrust=1.0,
                           lateral_thrust=1.0,
                           vertical_thrust=0.0)
        thruster[4] = dict(roll=-1.0,
                           pitch=-1.0,
                           yaw=0.0,
                           thrust=0.0,
                           lateral_thrust=0.0,
                           vertical_thrust=1.0)
        thruster[5] = dict(roll=-1.0,
                           pitch=1.0,
                           yaw=0.0,
                           thrust=0.0,
                           lateral_thrust=0.0,
                           vertical_thrust=-1.0)
        thruster[6] = dict(roll=1.0,
                           pitch=-1.0,
                           yaw=0.0,
                           thrust=0.0,
                           lateral_thrust=0.0,
                           vertical_thrust=-1.0)
        thruster[7] = dict(roll=1.0,
                           pitch=1.0,
                           yaw=0.0,
                           thrust=0.0,
                           lateral_thrust=0.0,
                           vertical_thrust=1.0)

        return thruster


def main():
    node = MixerNode()
    node.run()


if __name__ == "__main__":
    main()
