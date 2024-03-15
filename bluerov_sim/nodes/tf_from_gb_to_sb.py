#!/usr/bin/env python
import rospy
import threading
import numpy as np
# from std_msgs.msg import Float64
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_multiply
from tf.transformations import inverse_matrix




class gb_to_sb():
    def __init__(self):
        rospy.init_node("gb_to_sb")

        self.data_lock = threading.RLock()
        self.bluerov_pose = None
        self.mallard_pose = None
        self.slam_pose = None
        rospy.Subscriber("/bluerov/ground_truth/state", Odometry, self.bluerov_callback)
        rospy.Subscriber("/mallard/ground_truth/state", Odometry, self.mallard_callback)
        rospy.Subscriber("/slam_out_pose", PoseStamped, self.slam_callback)
        self.bluerov_slam_pub = rospy.Publisher("/bluerov/slam_pose", PoseStamped, queue_size=10)

        self.br = tf2_ros.TransformBroadcaster()

    def bluerov_callback(self, msg):
        with self.data_lock:
            self.bluerov_pose = msg.pose.pose

    def mallard_callback(self, msg):
        with self.data_lock:
            self.mallard_pose = msg.pose.pose

    def slam_callback(self, msg):
        with self.data_lock:
            self.slam_pose = msg.pose

    def calculate_bluerov_slam_pose(self):
        with self.data_lock:
            if self.bluerov_pose and self.slam_pose:
                # Convert pose to transformation matrices
                slam_mat = quaternion_matrix([self.slam_pose.orientation.x,
                                            self.slam_pose.orientation.y,
                                            self.slam_pose.orientation.z,
                                            self.slam_pose.orientation.w])
                slam_mat[0:3, 3] = [self.slam_pose.position.x, 
                                    self.slam_pose.position.y, 
                                    self.slam_pose.position.z]
                
                ground_mat = quaternion_matrix([self.mallard_pose.orientation.x,
                                            self.mallard_pose.orientation.y,
                                            self.mallard_pose.orientation.z,
                                            self.mallard_pose.orientation.w])
                ground_mat[0:3, 3] = [self.mallard_pose.position.x, 
                                    self.mallard_pose.position.y, 
                                    self.mallard_pose.position.z]
                
                inverse_ground_mat = inverse_matrix(slam_mat)
                ground_to_slam_mat = np.dot(inverse_ground_mat, ground_mat)

                bluerov_mat = quaternion_matrix([self.bluerov_pose.orientation.x,
                                                self.bluerov_pose.orientation.y,
                                                self.bluerov_pose.orientation.z,
                                                self.bluerov_pose.orientation.w])
                bluerov_mat[0:3, 3] = [self.bluerov_pose.position.x, 
                                    self.bluerov_pose.position.y, 
                                    self.bluerov_pose.position.z]

                # Inverse SLAM matrix to get map to SLAM transform
                inverse_slam_mat = tf.transformations.inverse_matrix(ground_to_slam_mat)

                # Apply transformation
                bluerov_slam_mat = inverse_slam_mat.dot(bluerov_mat)

                # Extract pose from the matrix
                bluerov_slam_pose = PoseStamped()
                bluerov_slam_pose.pose.position.x = bluerov_slam_mat[0, 3]
                bluerov_slam_pose.pose.position.y = bluerov_slam_mat[1, 3]
                bluerov_slam_pose.pose.position.z = bluerov_slam_mat[2, 3]
                bluerov_slam_quat = tf.transformations.quaternion_from_matrix(bluerov_slam_mat)
                bluerov_slam_pose.pose.orientation.x = bluerov_slam_quat[0]
                bluerov_slam_pose.pose.orientation.y = bluerov_slam_quat[1]
                bluerov_slam_pose.pose.orientation.z = bluerov_slam_quat[2]
                bluerov_slam_pose.pose.orientation.w = bluerov_slam_quat[3]

                return bluerov_slam_pose
    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            bluerov_slam_pose = self.calculate_bluerov_slam_pose()
            if bluerov_slam_pose:
                # Broadcast the transform or perform further processing
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = "base_link_b"
                self.bluerov_slam_pub.publish(bluerov_slam_pose)

                t.transform.translation.x = bluerov_slam_pose.pose.position.x
                t.transform.translation.y = bluerov_slam_pose.pose.position.y
                t.transform.translation.z = bluerov_slam_pose.pose.position.z
                t.transform.rotation.x = bluerov_slam_pose.pose.orientation.x
                t.transform.rotation.y = bluerov_slam_pose.pose.orientation.y
                t.transform.rotation.z = bluerov_slam_pose.pose.orientation.z
                t.transform.rotation.w = bluerov_slam_pose.pose.orientation.w

                self.br.sendTransform(t)
            rate.sleep()

def main():
    node = gb_to_sb()
    node.run()


if __name__ == "__main__":
    main()
