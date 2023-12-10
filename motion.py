from asyncore import write
import imp
from multiprocessing.connection import wait
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import math
from localization import Localization
import numpy as np
import utm
import csv
import time
import exsend2

class Motion(Node):
	#robot = None
	
	def __init__(self):
		super().__init__('motion') # type: ignore
		self._publisher = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
		self.robot=Localization()
		while not (self.robot.local_deg and self.robot.battery): #and self.robot.yawdeg			
			print("waiting")
			rclpy.spin_once(self.robot)
		
		self.robot.reset()
		rclpy.spin_once(self.robot)
		print(self.robot.global_deg)
		self.udp = exsend2.UDP()


		print("oki")
	
	def create_twist(self, lin_x=0.0, lin_y=0.0, lin_z=0.0, ang_x=0.0, ang_y=0.0, ang_z=0.0):
		twist = Twist()
		twist.linear.x = lin_x
		twist.linear.y = lin_y
		twist.linear.z = lin_z
		twist.angular.x = ang_x
		twist.angular.y = ang_y
		twist.angular.z = ang_z
		return twist
	
	def stop_twist(self):
		twist = self.create_twist()
		while True:
			self._publisher.publish(twist)

	def forward(self, speed_x, speed_z):
		twist = self.create_twist(lin_x = speed_x, ang_z=speed_z)
		self._publisher.publish(twist)
		rclpy.spin_once(self.robot)

	def turn(self, degree, speed = 0.5):
		self.robot.reset()
		deg = self.robot.local_deg
		print('locdeg ', deg)
		speed = speed*-1 if deg > 0 else speed
		
		# while abs(self.robot.local_deg) < abs(degree):
		while abs(float(self.udp.Receive(2023)[0])) < abs(degree):
			self.forward(0., speed)
			print('locdeg ', self.robot.local_deg, ' globaldeg: ', self.robot.global_deg)
			
		self.forward(0., 0.)
		finish_deg = self.robot.local_deg
		print(finish_deg)
		print('Correction')
		while abs(self.robot.local_deg) > abs(degree):
			self.forward(0., (0.2 if speed < 0 else -0.2))
			print('locdeg ', self.robot.local_deg, ' globaldeg: ', self.robot.global_deg)
		self.forward(0., 0.)

		print(self.robot.local_deg)

def main():
	rclpy.init(args=None)
	rosbot=Motion()
	print('Töltöttség :', round(rosbot.robot.battery, 2), '%')
	# for i in range(3):
	# 	print(3-i)
	# 	time.sleep(1)
	
	try:
		rosbot.turn(90, 0.5)
		#rosbot.turn(45, -0.7)
	except KeyboardInterrupt:
		rosbot.forward(0., 0.)
		rosbot.destroy_node()
		rclpy.shutdown()
		

if __name__ == "__main__":
	main()
