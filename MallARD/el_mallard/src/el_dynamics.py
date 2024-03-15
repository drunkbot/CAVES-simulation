#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist

class BodyToWheelVel():

	def __init__(self):
		self.sub = rospy.Subscriber('mallard/cmd_vel', Twist, self.callback)	
		self.pub = rospy.Publisher('mallard/cmd_wheel', Twist, queue_size=1)

		self.Vel_Mode = True
		self.cmd_vel = Twist()
		
		self.ctrl_c = False
		self.rate = rospy.Rate(100) #100Hz loop in order to have deltaT=10ms
		rospy.on_shutdown(self.shutdownhook)

		# parameters for el-mallard
        # Using parameters from its own/private namespace
		self.param = dict(R = rospy.get_param('~cfg_R'),
                          lx = rospy.get_param('~cfg_lx'),
                          ly = rospy.get_param('~cfg_ly'), 
                          M = rospy.get_param('~cfg_M'),
                          I = rospy.get_param('~cfg_I'),
                          Cu = rospy.get_param('~cfg_Cu'),
						  Cv = rospy.get_param('~cfg_Cv'),
						  Cr = rospy.get_param('~cfg_Cr'),
						  Cu_l = rospy.get_param('~cfg_Cu_l'),
						  Cv_l = rospy.get_param('~cfg_Cv_l'),
						  Cr_l = rospy.get_param('~cfg_Cr_l'),
                          deltaT = rospy.get_param('~cfg_deltaT'))  
		
		# Radius of the wheel in m
		self.R = self.param['R']
		#Distance from wheel center to x axis from body fram in m
		self.lx = self.param['lx']
		#Distance from wheel center to y axis from body frame in m
		self.ly = self.param['ly']
		#Mass in kg
		self.M = self.param['M']
		#moment of inertia in kgm^2
		self.I = self.param['I']
		#Non linear drag coeff in x
		self.Cu = self.param['Cu']
		#Non linear drag coeff in y
		self.Cv = self.param['Cv']
		#Non linear drag coeff for rotation
		self.Cr = self.param['Cr']
		#linear drag coeff in x
		self.Cu_l = self.param['Cu_l']
		#linear drag coeff in y
		self.Cv_l = self.param['Cv_l']
		#linear drag coeff for rotation
		self.Cr_l = self.param['Cr_l']
		#Value used to calculate the angle of rotation. Chosen small to approximate dt-from a derivative
		self.deltaT = self.param['deltaT']

		#Variables that hold the input Velocities/Forces - depending on the mode
		self.Vx = 0
		self.Vy = 0
		self.Wz = 0
		#Variables that hold the velocities for boat mode
		self.BoatWz = 0
		self.BoatVx = 0
		self.BoatVy = 0

		#Initialise mode and wheel_velocity. Needed in order to use the variables in the callback
		#A list that will hold the values for the wheel angular velocities/wheel forces for mode 0
		self.wheel_velocity = [0,0,0,0]	

		#Wheel of Twist message type.
		self.Wheel = Twist()
		

	def publish_wheel(self):
		while not self.ctrl_c:
			self.pub.publish(self.Wheel)
			break


	def shutdownhook(self):
		self.stop_elmallard()
		self.ctrl_c = True

	def stop_elmallard(self):
		#Stop the wheels
		rospy.loginfo('Stopping El-MallARD..')

		#Reset wheel vels to 0
		self.Wheel = Twist() 
		self.publish_wheel()
	

	#Function that transforms body frame velocities in x y and angular to wheel angular velocities
	def transform_velocities(self, V_x, V_y, W_z):
		self.wheel_velocity[0]= (V_x -V_y -(self.lx +self.ly) *W_z) /self.R
		self.wheel_velocity[1]=-(V_x +V_y +(self.lx +self.ly) *W_z) /self.R #negative sign because motor B and D are oppositely positioned compared to A and C.
		self.wheel_velocity[2]= (V_x +V_y -(self.lx +self.ly) *W_z) /self.R
		self.wheel_velocity[3]=-(V_x -V_y +(self.lx +self.ly) *W_z) /self.R #negative sign because motor B and D are oppositely positioned compared to A and C.
		return self.wheel_velocity

	def callback(self, twist):
		self.cmd_vel = twist #for checking cmd_vel values

		#Switch between Boat Mode and Velocity Mode
		#when L1 switch is pushed on the joystick
		L1 = twist.angular.y
		R1 = twist.angular.x

		#L1 -> Velocity Mode
		#R1 -> Boat Mode
		if L1 == 1:
			# self.Vel_Mode = not self.Vel_Mode
			self.Vel_Mode = True
		elif R1 == 1:
			self.Vel_Mode = False

		#X -> Stop the wheels completely. In any mode.
		pause = twist.linear.z #X button on the joystick
		if pause == 1: 
			self.BoatVx = 0
			self.BoatVy = 0
			self.BoatWz = 0

		#Set Body frame velocities for Velocity Mode
		elif self.Vel_Mode: #if == True 
			self.Vx = twist.linear.x/4
			self.Vy = twist.linear.y/4
			self.Wz = twist.angular.z/2

		#Set Body frame velocities for Boat Mode
		else: 
			self.Vx = twist.linear.x*2
			self.Vy = twist.linear.y*2
			self.Wz = twist.angular.z/2

	def body_to_wheel_vel(self):
		while not self.ctrl_c:
			#Set Wheel Vels to 0 (pause) to avoid sudden unintended acceleration when Wheel Vels are too low.
			wheel_vel_abs_sum = sum(map(abs, self.wheel_velocity))
			if wheel_vel_abs_sum < 1.0e-5:
				self.BoatVx = 0
				self.BoatVy = 0
				self.BoatWz = 0

			if self.Vel_Mode: #if == True
				#Velocity Control Mode

				self.wheel_velocity=self.transform_velocities(self.Vx, self.Vy, self.Wz)	
				#Assign them to a variable of the message type and publish them
				self.Wheel.linear.x=self.wheel_velocity[0]
				self.Wheel.linear.y=self.wheel_velocity[1]
				self.Wheel.linear.z=self.wheel_velocity[2]
				self.Wheel.angular.x=self.wheel_velocity[3]
				self.Wheel.angular.y=2

			else:
				#Allocate velocities for Boat mode. Here Vx,Vy represent forces and Wz is torque
				#This mode is done in the loop, because the velocities depend not only on current input,but
				#also on the previous ones

				#Calculate the yaw angle using the angular velocity of the boat
				Angle=self.BoatWz*self.deltaT

				#body frame velocities to inertial frame
				dummy = self.BoatVx
				self.BoatVx=self.BoatVx*math.cos(Angle)+self.BoatVy*math.sin(Angle)
				self.BoatVy=self.BoatVy*math.cos(Angle)-dummy*math.sin(Angle)
						
				#Calculate next velocities by using current ones, input forces and drag
				self.BoatVx=self.BoatVx+(((self.Cu*self.BoatVx*abs(self.BoatVx)-self.Cu_l*self.BoatVx+self.Vx)*self.deltaT)/self.M)
				self.BoatVy=self.BoatVy+(((self.Cv*self.BoatVy*abs(self.BoatVy)-self.Cv_l*self.BoatVy+self.Vy)*self.deltaT)/self.M)
				self.BoatWz=self.BoatWz+(((self.Cr*self.BoatWz*abs(self.BoatWz)-self.Cr_l*self.BoatWz+self.Wz)*self.deltaT)/self.I)

				#Find the velocity of each wheel
				self.wheel_velocity=self.transform_velocities(self.BoatVx,self.BoatVy,self.BoatWz)	
				#Assign them to a variable of the message type and publish them
				self.Wheel.linear.x=self.wheel_velocity[0]
				self.Wheel.linear.y=self.wheel_velocity[1]
				self.Wheel.linear.z=self.wheel_velocity[2]
				self.Wheel.angular.x=self.wheel_velocity[3]
				self.Wheel.angular.y=2
			
			#Publish wheel velocities via 'mallard/cmd_wheel' node
			self.publish_wheel()
			#Delay
			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('el_dynamics_node')
	bodytowheelvel_object = BodyToWheelVel()
	try:
		bodytowheelvel_object.body_to_wheel_vel()
	except rospy.ROSInterruptException:
		pass
