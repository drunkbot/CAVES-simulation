#!/usr/bin/env python
import rospy
import kglocal
import kguseful
import math
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from dynamic_reconfigure.server import Server
from mallard_autonomy.cfg import MallardPlannerParamsConfig
# 


class LocalPlanner:
    # ------ initialisation
    def __init__(self):
        # ------ initial parameters
        self.flag_first = True                                               # sets flag for first goal
        self.flag_goal_met = False  # sets the flag when rviz nav goal button clicked
        self.flag_end = False
        self.flag_final_goal_met = False
        self.n_goals = 0
        self.param = dict(vel=0.2, psivel=0.2, goal_tol=0.02, goal_tol_psi=0.1, t_ramp=2)  # params for el_mal
        self.goal_array = np.array([])
        self.x0 = self.y0 = self.x_goal = self.y_goal = 0
        self.q0 = self.q_goal = [0, 0, 0, 1]
        self.t0 = self.t_goal = self.t_goal_psi = 0
        self.ed = 0

        # ------ publishers and subscribers --------
        self.desired_publisher = rospy.Publisher("desired_state", PoseArray, queue_size=1)
        rospy.Subscriber("/slam_out_pose", PoseStamped, self.callslam)
        rospy.Subscriber('/path_poses', PoseArray, self.path_callback, queue_size=1)
        self.dynrecon = Server(MallardPlannerParamsConfig, self.dyn_recon_callback)

        # ------ rate and shutdown ------
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    # ------ publisher def ------
    def publish_des_state(self, des_state):
        while not self.ctrl_c:
            self.desired_publisher.publish(des_state)
            break

    # ----- shutdown stuff ---------
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop()
        self.ctrl_c = True

    def stop(self):
        rospy.loginfo("local planner shutdown hook")
        self.goal_array = np.array([])
        self.publish_des_state(PoseArray())  # publish zero PoseArray command when cancel is pressed
        # put safe shutdown commands here

    # ------- subscriber callbacks ---------
    # THIS CODE ADDED BY A WEST FOR RVIZ INTERFACE AND DYNAMIC RECONFIGURE
    def path_callback(self, msg):  # Manage inbound arrays of goal positions for coverage "lawnmower"
        # Ensure no previous goal positions are held by starting with blank array
        self.n_goals = 0
        self.flag_first = True
        self.flag_goal_met = False      # sets the flag when rviz nav goal button clicked
        self.flag_final_goal_met = False
        if len(msg.poses) == 0:         # an empty msg.poses message is sent when you press cancel in Rviz
            self.goal_array = np.array([])
            self.publish_des_state(PoseArray())   # publish zero PoseArray command when cancel is pressed
            return
        self.goal_array = np.empty([len(msg.poses), 3])
        # For every goal position, translate from geometry_msgs/Pose messages to [x,y,z] and add to goal_array
        for idx, position in enumerate(msg.poses):
            self.goal_array[idx, :] = kguseful.poseparse(position)

    def dyn_recon_callback(self, config, level):
        self.param['vel'] = config.lin_vel  # set linear velocity
        self.param['psivel'] = config.psi_vel  # set angular velocity
        self.param['goal_tol'] = config.gtol  # set linear goal tolerance
        self.param['goal_tol_psi'] = config.psi_gtol  # set angular goal tolerance
        # rospy.loginfo("linvel: %s", self.param['vel'])
        rospy.loginfo("""Reconfigure Request from local_planner: {lin_vel}, {psi_vel}, {gtol}, {psi_gtol}""".format(**config))
        return config
    # ----- END CODE ADDED BY A WEST --------

    # local planer callback
    def callslam(self, slam_out):
        # if no goal positions exist in goal_array, then exit this callback by using return
        # no further computation is undertaken
        if len(self.goal_array) == 0:
            return

        q_now = [slam_out.pose.orientation.x, slam_out.pose.orientation.y, slam_out.pose.orientation.z,
                 slam_out.pose.orientation.w]

        # if it's the first run then zero is current position
        if self.flag_first:
            self.x0 = slam_out.pose.position.x
            self.y0 = slam_out.pose.position.y
            self.q0 = q_now
            self.x_goal = self.goal_array[self.n_goals, 0]
            self.y_goal = self.goal_array[self.n_goals, 1]
            self.q_goal = tft.quaternion_from_euler(0, 0, self.goal_array[self.n_goals, 2])

        # if a goal has been met then increment the goal    # sleep(1)
        if self.flag_goal_met:
            self.x0 = self.goal_array[self.n_goals, 0]
            self.y0 = self.goal_array[self.n_goals, 1]
            self.q0 = tft.quaternion_from_euler(0, 0, self.goal_array[self.n_goals, 2])
            self.n_goals = self.n_goals + 1
            self.x_goal = self.goal_array[self.n_goals, 0]
            self.y_goal = self.goal_array[self.n_goals, 1]
            self.q_goal = tft.quaternion_from_euler(0, 0, self.goal_array[self.n_goals, 2])

        # work out time it will take to get to new goal, xy and psi
        if self.flag_first or self.flag_goal_met:
            self.t0 = slam_out.header.stamp.secs + slam_out.header.stamp.nsecs * 0.000000001
            dist = math.sqrt(pow((self.x_goal - self.x0), 2) + pow((self.y_goal - self.y0), 2))
            self.t_goal = kguseful.safe_div(dist, self.param['vel'])  # avoid zero division stability
            self.ed = kguseful.err_psi_fun(self.q0, self.q_goal)
            self.t_goal_psi = abs(kguseful.safe_div(self.ed, self.param['psivel']))
            self.flag_first = False
            self.flag_goal_met = False
            # rospy.loginfo("t_goal_psi: %s, t_goal: %s", t_goal_psi, t_goal)
            # rospy.loginfo("x: %s, y: %s", data.pose.position.x, data.pose.position.y)
            # rospy.loginfo("xg: %s, yg: %s", x_goal, y_goal)
            rospy.loginfo("goal number: %s, end goal: %s", self.n_goals + 1, self.goal_array.shape[0])

        # get current desired positions and velocities
        t_now = (slam_out.header.stamp.secs + slam_out.header.stamp.nsecs * 0.000000001) - self.t0  # since goal 0
        xvelmax = abs(kguseful.safe_div((self.x_goal - self.x0), self.t_goal))
        yvelmax = abs(kguseful.safe_div((self.y_goal - self.y0), self.t_goal))
        xdes, xveldes = kglocal.velramp(t_now, xvelmax, self.x0, self.x_goal, self.param['t_ramp'])
        ydes, yveldes = kglocal.velramp(t_now, yvelmax, self.y0, self.y_goal, self.param['t_ramp'])
        qdes = kglocal.despsi_fun(self.q_goal, self.t_goal_psi, self.q0, t_now)
        psiveldes = kglocal.desvelpsi_fun(self.ed, self.t_goal_psi, t_now, self.param['psivel'])

        # publish desired state
        des_state = PoseArray()
        pose0 = Pose()
        pose0.position.x = xdes
        pose0.position.y = ydes
        pose0.orientation.z = qdes[2]
        pose0.orientation.w = qdes[3]
        des_state.poses.insert(0, pose0)                # compile into pose array
        pose1 = Pose()
        pose1.position.x = xveldes
        pose1.position.y = yveldes
        pose1.orientation.z = psiveldes
        des_state.poses.insert(1, pose1)
        self.publish_des_state(des_state)

        # if goal is met then move to next goal
        if abs(self.x_goal - slam_out.pose.position.x) <= self.param['goal_tol']:
            if abs(self.y_goal - slam_out.pose.position.y) <= self.param['goal_tol']:
                e_psi = kguseful.err_psi_fun(q_now, self.q_goal)
                if abs(e_psi) <= self.param['goal_tol_psi']:
                    if self.goal_array.shape[0] != self.n_goals + 1:  # if there are more goals
                        print 'goal met'
                        self.flag_goal_met = True  # set flag to move to next goal
                    if self.goal_array.shape[0] == self.n_goals + 1:  # if there are no more goals
                        if not self.flag_final_goal_met:
                            print 'final goal met - holding position'
                            self.flag_final_goal_met = True


if __name__ == '__main__':
    rospy.init_node('local_planner', anonymous=True)  # initialise node "local control"
    try:
        LocalPlanner()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
