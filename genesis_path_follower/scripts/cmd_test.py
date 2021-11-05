#!/usr/bin/env python
import rospy
import math as m
import numpy as np
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import Imu
from genesis_msgs.msg import SteeringReport
from genesis_path_follower.msg import state_est
from std_msgs.msg import Float64

''' Main Class to Publish Vehicle State Information '''
class ctrl_test(object):
	def __init__(self):
		rospy.init_node('ctrl_test', anonymous=True)	

		self.st_cmd_pub = rospy.Publisher('/Steering_Command',Float64,queue_size=1)
		rospy.Subscriber('/ctrl_feedback', TwistStamped, self.ctrl_feedback_callback, queue_size=1)

	
		#  -610  -305  0
		# 	-710  	   100
		# self.target_data = [-300, -400, -500, -600, -710, -610, -510, -410, -310, -210, -110, 0, 100, -100, -300, -500, -700, -500, -300, -100, 100, 0, -100, -200, -300 ]\
		self.target_data = [-310, -210, -110, 0, 100, -100, -300, -500, -700, -500, -300, -100, 100, 0, -100, -200, -300 ]
		self.count = 0
	def ctrl_feedback_callback(self,msg):
		cur_steer = msg.twist.angular.z
		pub_data = Float64()
		pub_data.data = self.target_data[self.count]
		self.st_cmd_pub.publish(pub_data)	
		diff = abs(cur_steer - self.target_data[self.count])
		print(diff)	
		print()
		if  diff < 3: 
			rospy.sleep(5) 
			self.count = self.count+1
		
		if self.count == len(self.target_data)-1: 
			self.count = 0
			
if __name__=='__main__':
	try:
		ctrl_test()
	except rospy.ROSInterruptException:
		pass
	rospy.spin()
