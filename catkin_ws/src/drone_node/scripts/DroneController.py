#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
import math

pi = math.pi
pi_2 = pi / 2.0


class DroneController():
	"""
	A simple object to help interface with mavros
	"""

	def __init__(self):
		rospy.init_node("drone_control_node")
		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
		rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

		self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
		self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
		self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

		# mode 0 = STABILIZE
		# mode 4 = GUIDED
		# mode 9 = LAND
		self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

		self.rc = RCIn()
		self.pose = Pose()
		self.timestamp = rospy.Time()

# Keep track of the current manual RC values
	def rc_callback(self, data):
		self.rc = data

#Handle local position information
	def pose_callback(self, data):
		self.timestamp = data.header.stamp
		self.pose = data.pose

#  setpoint setting. perhaps not using gps for mission flight
	def goto(self, pose):
		"""
		Set the given pose as a the next setpoint by sending
		a SET_POSITION_TARGET_LOCAL_NED message. The copter must
		be in GUIDED mode for this to work.
		"""
		pose_stamped = PoseStamped()
		pose_stamped.header.stamp = self.timestamp
		pose_stamped.pose = pose

		self.cmd_pos_pub.publish(pose_stamped)

	def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
		pose = Pose()
		pose.position.x = x
		pose.position.y = y
		pose.position.z = z

		quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

		pose.orientation.x = quat[0]
		pose.orientation.y = quat[1]
		pose.orientation.z = quat[2]		pose.orientation.w = quat[3]
		self.goto(pose)
		# print(quat)

	def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
		"""
		Send comand velocities. Must be in GUIDED mode. Assumes angular
		velocities are zero by default.
		"""
		cmd_vel = Twist()

		cmd_vel.linear.x = vx
		cmd_vel.linear.y = vy
		cmd_vel.linear.z = vz

		cmd_vel.angular.x = avx
		cmd_vel.angular.y = avy
		cmd_vel.angular.z = avz

		self.cmd_vel_pub.publish(cmd_vel)

# Arm the throttle
	def arm(self):
		return self.arm_service(True)

# Disarm the throttle
	def disarm(self):
		return self.arm_service(False)

# Arm the throttle, takeoff to a few feet, and set to guided mode
	def takeoff(self, height=1.0):
		# Set to stabilize mode for arming
		#mode_resp = self.mode_service(custom_mode="0")
		mode_resp = self.mode_service(custom_mode="16")
		self.arm()

		# Set to guided mode
		#mode_resp = self.mode_service(custom_mode="4")

		# Takeoff
		takeoff_resp = self.takeoff_service(altitude=height)

		# return takeoff_resp
		return mode_resp

# Set in LAND mode, which should cause the UAV to descend directly, land, and disarm.
	def land(self):
		resp = self.mode_service(custom_mode="9")
		self.disarm()

# simple demo1
	def simple_demo_1(self):
		"""
		A simple demonstration of using mavros commands to control a UAV.
		"""
		c = MavController()
		rospy.sleep(1)

		print("Takeoff")
		c.takeoff(0.5)
		rospy.sleep(3)
		c.goto_xyz_rpy(0, 0, 1.2, 0, 0, 0)
		rospy.sleep(3)

		print("Waypoint 1: position control")
		c.goto_xyz_rpy(0.0, 0.0, 1.2, 0, 0, -1*pi_2)
		rospy.sleep(2)
		c.goto_xyz_rpy(0.4, 0.0, 1.2, 0, 0, -1*pi_2)
		rospy.sleep(3)
		print("Waypoint 2: position control")
		c.goto_xyz_rpy(0.4, 0.0, 1.2, 0, 0, 0)
		rospy.sleep(2)
		c.goto_xyz_rpy(0.4, 0.4, 1.2, 0, 0, 0)
		rospy.sleep(3)
		print("Waypoint 3: position control")
		c.goto_xyz_rpy(0.4, 0.4, 1.2, 0, 0, pi_2)
		rospy.sleep(2)
		c.goto_xyz_rpy(0.0, 0.4, 1.2, 0, 0, pi_2)
		rospy.sleep(3)
		print("Waypoint 4: position control")
		c.goto_xyz_rpy(0.0, 0.4, 1.2, 0, 0, 2*pi_2)
		rospy.sleep(2)
		c.goto_xyz_rpy(0.0, 0.0, 1.2, 0, 0, 2*pi_2)
		rospy.sleep(3)

		#print("Velocity Setpoint 1")
		# c.set_vel(0,0.1,0)
		# rospy.sleep(5)
		#print("Velocity Setpoint 2")
		# c.set_vel(0,-0.1,0)
		# rospy.sleep(5)
		#print("Velocity Setpoint 3")
		# c.set_vel(0,0,0)
		# rospy.sleep(5)

		print("Landing")
		c.land()

#simple demo2
	def simple_demo_2(self):
		"""
		A simple demonstration of using mavros commands to control a UAV.
		"""
		c = MavController()
		rospy.sleep(1)

		print("Takeoff")
		c.takeoff(1.0)
		rospy.sleep(7)

		r = 1
		circle_center_x = 0.0
		circle_center_y = 0.0
		circle_height = 1.0

		c.goto_xyz_rpy(circle_center_x, circle_center_y, circle_height, 0, 0, 0)
		rospy.sleep(3)
		c.goto_xyz_rpy(circle_center_x + r, circle_center_y, circle_height, 0, 0, 0)
		rospy.sleep(3)

		# for i in range(10):
		#     y = i * 0.2 / 10.0
		#     c.goto_xyz_rpy(0.2 + r, y, 1.2, 0, 0, 0)
		#     rospy.sleep(0.2)

		for i in range(360):
			theta = i * 2.0 * pi / 180.0
			x = circle_center_x + r * math.cos(theta)
			y = circle_center_y + r * math.sin(theta)
			z = circle_height
			c.goto_xyz_rpy(x, y, z, 0.0, 0.0, theta)
			rospy.sleep(0.1)

		rospy.sleep(3)

		c.goto_xyz_rpy(0.0, 0.0, circle_height, 0, 0, 0)
		rospy.sleep(3)

		#print("Velocity Setpoint 1")
		# c.set_vel(0,0.1,0)
		# rospy.sleep(5)
		#print("Velocity Setpoint 2")
		# c.set_vel(0,-0.1,0)
		# rospy.sleep(5)
		#print("Velocity Setpoint 3")
		# c.set_vel(0,0,0)
		# rospy.sleep(5)

		print("Landing")
		c.land()

# create waypoint list  and push
	def create_waypoint(self, mission):
		self.waypoint_clear_client()
		wl = []
		for  waypoint in mission:
			wp = Waypoint()
			wp.frame = 3
			wp.command = 22  # takeoff
			wp.is_current = False
			wp.autocontinue = True
			wp.param1 = 0 
			wp.param2 = 0
			wp.param3 = 0
			wp.param4 = 0
			wp.x_lat = waypoint[0]
			wp.y_long = waypoint[1]
			wp.z_alt = 1
			wl.append(wp)

		try:
			service = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
			service(start_index=0, waypoints=wl)

			if service.call(wl).success:  
				print('write mission success')
			else:
				print('write mission error')

		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)

	def waypoint_clear_client():
		try:
			response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
			return response.call().success
		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)
			return False

'''
if __name__ == '__main__':
	rospy.init_node('waypoint_node', anonymous=True)
	pub = rospy.Publisher('global', String, queue_size=10)
	konum = "suan buradasin"  # to be used later
	pub.publish(konum)
	create_waypoint()
'''