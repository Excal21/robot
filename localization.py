
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState
import rclpy
from rclpy.node import Node
import numpy as np
import utm
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import transformations

class Localization(Node):
	#rear_pose = Pose()
	#front_pose = Pose()
	odom_theta = 0.0
	
	odom_pose = Pose()
 
	reset_deg = 0.0
   
	pose = Pose()
	#x: east, y: north
	theta = 0.0
	global_deg = 0.0
	local_deg = 0.0

	#gps_rear=False
	#gps_front=False
	odom=False
	yawdeg = False
	gnss = False

	velocity = Twist()

	ori = False
	abs_ori_deg = False
	battery = False

	def __init__(self):
		super().__init__('localization') # type: ignore
		self.imu_subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 1)
		self.odom_subscription = self.create_subscription(Pose, '/pose', self.odom_callback, 1)
		self.battery_subscription = self.create_subscription(BatteryState, '/battery', self.battery_callback, 1)
	def battery_callback(self, msg):
		self.battery = (msg.voltage - 8.4) / 4.2 * 100
		

	def imu_callback(self, msg):
		quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
		euler = transformations.transformations.euler_from_quaternion(quaternion)
		self.ori = math.degrees(euler[0])
		self.global_deg = self.ori
		
		self.loc_deg()
		
		#print('Direction ', self.ori)
		#print('Abs direction ', self.abs_ori_deg)

	def odom_callback(self, msg):
		# print("pose x = " + str(msg.pose.position.x))
		# print("pose y = " + str(msg.pose.position.y))
		# print("orientacion z = " + str(msg.pose.orientation.z))
		# print("orientacion w = " + str(msg.pose.orientation.w))
		self.pose.position.x = msg.pose.position.x
		self.pose.position.y = msg.pose.position.y
		quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
		euler = transformations.transformations.euler_from_quaternion(quaternion)
		# print(euler)
		# print(transformations.transformations.euler_from_quaternion((-0.014597449451684952, -0.029851989820599556, 0.9525320651430115, 0.3044382776104044)))
		self.odom_theta = euler[0]

		self.odom = True
		
	def odom2_callback(self, msg):

		self.pose.position.x = msg.pose.pose.position.x
		self.pose.position.y = msg.pose.pose.position.y
		self.odom = True
  
	def reset(self):	
		self.reset_deg = self.global_deg
		self.local_deg = 0.0


	def yawdeg_callback(self, msg):
		self.global_deg = msg.data
		self.theta = math.radians(msg.data)
		self.loc_deg()
		self.yawdeg= True

	def loc_deg(self):
		x = self.global_deg - self.reset_deg
		if x > 0:
			self.local_deg = x//180*-180+x%180
		else:
			self.local_deg = x//-180*180+x%-180
		#print(self.local_deg) 

	def pose_callback(self, msg):
		#print("pose x = " + str(msg.position.x))
		#print("pose y = " + str(msg.position.y))
		#print("pose z = " + str(msg.position.z))
		self.pose.position.x = msg.position.x
		self.pose.position.y = msg.position.y
		self.gnss = True
	
	def velocity_callback(self, msg):
		self.velocity.linear.x = msg.linear.x
		self.velocity.linear.y = msg.linear.y
		self.velocity.linear.z = msg.linear.z
		self.velocity.angular.x = msg.angular.x
		self.velocity.angular.y = msg.angular.y
		self.velocity.angular.z = msg.angular.z
		#print(msg.linear.x)

def main():
	rclpy.init(args=None)
	teszt=Localization()
	rclpy.spin_once(teszt)
	teszt.reset()
	rclpy.spin_once(teszt)
	try:
		while True:
			print('locdeg: ', teszt.local_deg, ' globdeg: ', teszt.global_deg)
			rclpy.spin_once(teszt)
	except KeyboardInterrupt:
		pass

	rclpy.spin_once(teszt)
	#rclpy.spin_once(teszt)

	teszt.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
