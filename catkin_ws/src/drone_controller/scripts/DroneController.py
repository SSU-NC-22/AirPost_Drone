#!/usr/bin/env python3

import rospy
#import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import math

pi = math.pi
pi_2 = pi / 2.0


class DroneController():
	"""
	A simple object to help interface with mavros
	"""

	def __init__(self):
		#rospy.init_node("drone_control_node")
		self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

		# mode 0 = STABILIZE
		# mode 4 = GUIDED
		# mode 9 = LAND
		self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

		self.rc = RCIn()

# Keep track of the current manual RC values
	def rc_callback(self, data):
		self.rc = data

# Arm the throttle
	def arm(self):
		return self.arm_service(True)

# Disarm the throttle
	def disarm(self):
		return self.arm_service(False)

# Arm the throttle, takeoff to a few feet, and set to guided mode
	def takeoff(self, height=1.0):
		# Set to loiter mode for arming
		mode_resp = self.mode_service(custom_mode="4")
		self.arm()

		# Takeoff
		takeoff_resp = self.takeoff_service.call(altitude=height)

		# return takeoff_resp
		return takeoff_resp
# Set mode
	def setmode(self, mode):
		return self.mode_service(custom_mode=str(mode))

# Set in LAND mode, which should cause the UAV to descend directly, land, and disarm.
	def land(self):
		resp = self.mode_service(custom_mode="9")
		self.disarm()

# create waypoint list  and push
	def create_waypoint(self, mission, tagidx):
		self.waypoint_clear_client()
		wl = []

		for  idx, waypoint in enumerate(mission):
			wp = Waypoint()
			wp.frame = 3
			wp.command = waypoint[3]  # takeoff
			wp.is_current = False
			wp.autocontinue = True
			wp.param1 = 0
			if idx == tagidx:
				wp.param1 = 10
			wp.param2 = 0
			wp.param3 = 0
			wp.param4 = 0
			wp.x_lat = waypoint[0]
			wp.y_long = waypoint[1]
			wp.z_alt = waypoint[2]
			wl.append(wp)

		try:
			service = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
			service(start_index=0, waypoints=wl)

			if service.call(waypoints=wl).success:  
				print('write mission success')
			else:
				print('write mission error')

		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)

	def waypoint_clear_client(self):
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

'''flight modes
enum class Number {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
    SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
    FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
    ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
    AUTOROTATE =   26,  // Autonomous autorotation
    NEW_MODE =     27,  // your new flight mode
};
'''