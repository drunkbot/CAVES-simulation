#!/usr/bin/env python
import rospy
import kglocal
import kguseful
import collections as coll
from geometry_msgs.msg import Twist, PoseStamped, PoseArray

class TrajControl:

    def __init__(self):
        # ------ initial parameters
        
        # parameters for el-mallard
        # Using parameters from its own/private namespace
        self.param = dict(kp = rospy.get_param('~cfg_kp'),
                          kd = rospy.get_param('~cfg_kd'),
                          kp_psi = rospy.get_param('~cfg_kp_psi'), 
                          kd_psi = rospy.get_param('~cfg_kd_psi'),
                          lim = rospy.get_param('~cfg_lim'),
                          lim_psi = rospy.get_param('~cfg_lim_psi'),
                          nv = rospy.get_param('~cfg_nv'))  

        self.dtv = coll.deque([1e-5, 1e-5], maxlen = self.param['nv'])                    # setup buffers
        self.dxv = coll.deque([1e-5, 1e-5], maxlen = self.param['nv'])
        self.dyv = coll.deque([1e-5, 1e-5], maxlen = self.param['nv'])
        self.dpsiv = coll.deque([1e-5, 1e-5], maxlen = self.param['nv'])
        self.x = 0
        self.y = 0
        self.q_now = [0, 0, 0, 1]
        self.xvel = 0
        self.yvel = 0
        self.psivel = 0
        self.tp = -0.05
        self.xp = 0
        self.yp = 0
        self.qp = [0, 0, 0, 0]
        self.pub_pause = 5
        self.t0 = rospy.get_time() - self.pub_pause

        # ------ publishers and subscribers --------
        self.thrust_pub = rospy.Publisher('/mallard/cmd_vel1', Twist, queue_size=10)
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.callslam)
        rospy.Subscriber('/desired_state', PoseArray, self.calldes, queue_size=1)

        # ------ rate and shutdown ------
        # self.rate = rospy.Rate(10)                                        # set the loop rate when sleep() is used
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    # ------ publisher def ------
    def publish_thrust(self, thrust):
        while not self.ctrl_c:
            self.thrust_pub.publish(thrust)
            break

    # ----- shutdown stuff ---------
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop()
        self.ctrl_c = True

    def stop(self):
        rospy.loginfo("trajectory controller shutdown hook")
        # put safe shutdown commands here
        self.thrust_pub.publish(Twist())

    # numerical differentiation
    def callslam(self, slampose):
        self.x = slampose.pose.position.x
        self.y = slampose.pose.position.y
        self.q_now = [slampose.pose.orientation.x, slampose.pose.orientation.y, slampose.pose.orientation.z,
                      slampose.pose.orientation.w]
        t_slam = (slampose.header.stamp.secs + slampose.header.stamp.nsecs * 0.000000001)

        self.dtv.appendleft(t_slam - self.tp)  # time difference vector
        self.dxv.appendleft(slampose.pose.position.x - self.xp)  # x difference vector
        self.dyv.appendleft(slampose.pose.position.y - self.yp)  # y difference vector
        dpsi = kguseful.err_psi_fun(self.qp, self.q_now)
        self.dpsiv.appendleft(dpsi)  # psi difference vector
        self.xvel = kglocal.vel_fun(list(self.dxv), list(self.dtv))  # velocity vectors x, y and psi
        self.yvel = kglocal.vel_fun(list(self.dyv), list(self.dtv))
        self.psivel = kglocal.vel_fun(list(self.dpsiv), list(self.dtv))

        # change current to previous values
        self.tp = t_slam
        self.xp = self.x
        self.yp = self.y
        self.qp = self.q_now

    # trajectory tracking controller
    def calldes(self, des_state):
        if len(des_state.poses) == 0:  # an empty des_state.poses message is sent when you press cancel in Rviz
            self.publish_thrust(Twist())
            return

        # unpack desired states
        xdes = des_state.poses[0].position.x
        ydes = des_state.poses[0].position.y
        qdes = [0, 0, des_state.poses[0].orientation.z, des_state.poses[0].orientation.w]
        xveldes = des_state.poses[1].position.x
        yveldes = des_state.poses[1].position.y
        psiveldes = des_state.poses[1].orientation.z

        # Get forces in nav frame using PD controller
        xf_nav = kglocal.cont_fun(self.x, xdes, self.xvel, xveldes, self.param['kp'], self.param['kd'],
                                  self.param['lim'])
        yf_nav = kglocal.cont_fun(self.y, ydes, self.yvel, yveldes, self.param['kp'], self.param['kd'],
                                  self.param['lim'])
        psif_nav = kglocal.contpsi_fun(self.q_now, qdes, self.psivel, psiveldes, self.param['kp_psi'],
                                       self.param['kd_psi'], self.param['lim_psi'])

        # put xy forces into body frame
        f_body = kguseful.quat_rot([xf_nav, yf_nav, 0], [-self.q_now[0], -self.q_now[1], -self.q_now[2], self.q_now[3]])
        # put forces into twist structure and publish
        thrust = Twist()
        thrust.linear.x = f_body[0]
        thrust.linear.y = f_body[1]
        thrust.angular.z = psif_nav  # minus needed to fix wrong direction on el_mal
        self.publish_thrust(thrust)


if __name__ == '__main__':
    rospy.init_node('trajectory_control', anonymous=True)  # initialise node
    try:
        TrajControl()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
