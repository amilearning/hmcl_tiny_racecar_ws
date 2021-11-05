# #!/usr/bin/env python
# import rospy
# import math as m
# import numpy as np
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from sensor_msgs.msg import NavSatFix
# from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped, PoseStamped
# from sensor_msgs.msg import Imu
# from genesis_msgs.msg import SteeringReport
# from genesis_path_follower.msg import state_est
# from autoware_msgs.msg import VehicleStatus
# from std_msgs.msg import Float64
# from nav_msgs.msg import Odometry






# # Vehicle State Publisher for the Hyundai Genesis.  Uses OxTS and vehicle CAN messages to localize.
# # You will need to add the package from https://github.com/MPC-Berkeley/genesis_msgs for the steering angle measurement.

# ''' GPS -> XY '''
# def latlon_to_XY(lat0, lon0, lat1, lon1):
# 	''' 
# 	Convert latitude and longitude to global X, Y coordinates,
# 	using an equirectangular projection.

# 	X = meters east of lon0
# 	Y = meters north of lat0
# 	lat0,1 in rad 
# 	lon0,1 in rad
	
# 	Sources: http://www.movable-type.co.uk/scripts/latlong.html
# 		     https://github.com/MPC-Car/StochasticLC/blob/master/controller.py
# 	'''
# 	# R_earth = 6371000 # meters
# 	# # delta_lat = m.radians(lat1 - lat0)	
# 	# # delta_lon = m.radians(lon1 - lon0)

# 	# delta_lat = lat1 - lat0	
# 	# delta_lon = lon1 - lon0
# 	# # lat_avg = 0.5 * ( m.radians(lat1) + m.radians(lat0) )
# 	# lat_avg = 0.5 * ( lat1 + lat0 )
# 	# X = R_earth * delta_lon * m.cos(lat_avg)
# 	# Y = R_earth * delta_lat
# 	X = lat1
# 	Y = lon1
# 	return X,Y

# ''' Take ROS timestamp in seconds from a message '''
# def extract_ros_time(msg):
# 	return msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

# ''' Check if timestamps are within thresh seconds of current time '''
# def is_time_valid(tm_now, tm_arr, thresh=0.05):
# 	for tm_rcvd in tm_arr:
# 		diff = m.fabs(tm_now - tm_rcvd)
# 		if diff > thresh: # seconds
# 			return False
# 	return True

# ''' Main Class to Publish Vehicle State Information '''
# class StatePublisher(object):
# 	def __init__(self):
# 		rospy.init_node('state_publisher', anonymous=True)	

# 		attrs = ['tm_gps', 'lat', 'lon', 'x', 'y', \
# 		         'tm_vel', 'v', 'v_long', 'v_lat', \
# 		         'tm_imu', 'psi', 'long_accel', 'lat_accel', 'yaw_rate' \
# 		         'tm_df', 'df']
# 		for attr in attrs:
# 			setattr(self, attr, None)
# 		self.tm_df  = None
# 		self.tm_gps = None
# 		self.tm_vel = None
# 		self.tm_vel_prev = None
# 		self.tm_imu = None
		
		
# 		if not (rospy.has_param('lat0') and rospy.has_param('lon0')):
# 			raise ValueError('Invalid rosparam global origin provided!')

# 		# if not rospy.has_param('time_check_on'):
# 		# 	raise ValueError('Did not specify if time validity should be checked!')

# 		self.LAT0 = rospy.get_param('lat0')
# 		self.LON0 = rospy.get_param('lon0')	
# 		# time_check_on = rospy.get_param('time_check_on')
		
# 		state_pub = rospy.Publisher('state_est', state_est, queue_size=1)

# 		# rospy.Subscriber('/fix', NavSatFix, self._parse_gps_fix, queue_size=1)
# 		rospy.Subscriber('/vesc/odom', Odometry, self.vel_callback, queue_size=1)
# 		rospy.Subscriber('/ndtpso_slam_front/pose', PoseStamped, self._pose_callback, queue_size= 1)
# 		# rospy.Subscriber('/vehicle_status', VehicleStatus, self.sim_st_angle_callback, queue_size= 1)

		
# 		rospy.Subscriber('/imu', Imu, self._parse_imu_data, queue_size=1)
# 		# rospy.Subscriber('/vehicle/steering', SteeringReport, self._parse_steering_angle, queue_size=1)
# 		# rospy.Subscriber('/ctrl_feedback', TwistStamped, self._parse_steering_angle, queue_size=1)
# 		rospy.Subscriber('/vesc/sensors/servo_position_command', Float64, self._parse_steering_angle, queue_size=1)
		
	
# 		r = rospy.Rate(100)
# 		while not rospy.is_shutdown():		
			
# 			if None in [self.tm_gps, self.tm_vel, self.tm_imu, self.tm_df]: 
# 				r.sleep() # If the vehicle state info has not been received.
# 				print("time is not aligned.")
# 				continue

# 			curr_state = state_est()
# 			curr_state.header.stamp = rospy.Time.now()
			
# 			# if time_check_on:
# 			# time_valid = is_time_valid(extract_ros_time(curr_state), \
# 			# 							[self.tm_gps, self.tm_vel, self.tm_imu, self.tm_df])
# 			time_valid = True
# 			if not time_valid:
# 				r.sleep()
# 				print("time is not aligned")
# 				continue

# 			curr_state.lat = self.lat
# 			curr_state.lon = self.lon

# 			curr_state.x   = self.x
# 			curr_state.y   = self.y
# 			curr_state.psi = self.psi
# 			curr_state.v   = self.v

# 			curr_state.v_long   = self.v_long
# 			curr_state.v_lat    = self.v_lat
# 			curr_state.yaw_rate = self.yaw_rate
			
# 			curr_state.a_long = self.long_accel
# 			curr_state.a_lat  = self.lat_accel
# 			curr_state.df     = self.df

# 			state_pub.publish(curr_state)
			
# 			r.sleep()


# 	def _pose_callback(self,msg):
		
# 		self.x = msg.pose.position.x
# 		self.y = msg.pose.position.y
# 		self.lat = self.x 
# 		self.lon = self.y

# 		orientation_q = msg.pose.orientation
# 		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
# 		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
# 		self.psi = yaw
# 		self.tm_gps = extract_ros_time(msg)

# 	# def _parse_gps_fix(self, msg):
# 	# 	# This function gets the latitude and longitude from the OxTS and does localization.		
# 	# 	self.lat    = msg.latitude
# 	# 	self.lon    = msg.longitude
# 	# 	self.x, self.y = latlon_to_XY(self.LAT0, self.LON0, self.lat, self.lon)

# 	# 	self.tm_gps = extract_ros_time(msg)

		
# 	def vel_callback(self,msg):
		
# 		self.v = msg.twist.twist.linear.x # Could set a lock here.  This is copy by value so should be okay.
# 		self.yaw_rate = msg.twist.twist.angular.z 
# 		if self.psi is None:
# 			return
# 		self.v_lat = m.cos(self.psi)*self.v
# 		self.v_long = m.sin(self.psi)*self.v	
		
# 		# if self.tm_vel is not None and self.tm_vel_prev is not None: 
# 		# 	dt = (self.tm_vel - self.tm_vel_prev)
# 		# 	self.lat_accel = (self.v_lat - self.v_lat_prev) / (dt+1e-9)
# 		# 	self.long_accel = (self.v_long - self.v_long_prev) / (dt+1e-9)
# 		# else:
# 		# 	self.long_accel = 0.0
# 		# 	self.lat_accel = 0.0
# 		self.tm_vel =  extract_ros_time(msg)

# 		self.tm_vel_prev = self.tm_vel
# 		self.v_lat_prev = self.v_lat
# 		self.v_long_prev =self.v_long
		


	

# 	# def sim_st_angle_callback(self, msg):  
# 	# 	self.df = msg.angle  # -1*(msg.twist.angular.z+363.33) / 12.456 * 3.14195 / 180  # (rad)		
		
# 	# 	self.tm_df = extract_ros_time(msg)

# 	def _parse_imu_data(self, msg):
# 		# This function gets the yaw angle and acceleration/yaw rate from the OXTS.		
		
# 		# Get acceleration and yaw rate information.  The y-axis of the OXTS points right.
# 		self.long_accel =  msg.linear_acceleration.x # m/s^2
# 		self.lat_accel  = -msg.linear_acceleration.y # m/s^2
# 		self.yaw_rate   =  msg.angular_velocity.z    # rad/s
		
# 		if self.tm_vel is not None:
# 			self.tm_imu = self.tm_vel
		
		
# 	def _parse_steering_angle(self, msg):  
		
# 		steering_angle_to_servo_gain = -1.0535
# 		steering_angle_to_servo_offset = 0.5604
# 		# self.df = -1*(msg.twist.angular.z+363.33) / 12.456 * 3.14195 / 180  # (rad)		
# 		self.df = (msg.data - steering_angle_to_servo_offset)
# 		if self.df is not 0:
# 			self.df = self.df / steering_angle_to_servo_gain
# 		# self.tm_df = rospy.Time.now()
# 		if self.tm_vel is not None:
# 			self.tm_df = self.tm_vel


# if __name__=='__main__':
# 	try:
# 		StatePublisher()
# 	except rospy.ROSInterruptException:
# 		pass
