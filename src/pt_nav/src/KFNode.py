#!/usr/bin/env python
#Kalman Filter Node
#@file KFNode.py

#@author Rinoy Pazhekattu (rinoy@rinoyp.com)
#@par Creation Date: 2016-08-01

#@par Copyrights:
#Copyright (c) 2016, Rinoy Pazhekattu.
#All rights reserved.

import math
import numpy as np
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Imu,NavSatFix,NavSatStatus
from geometry_msgs.msg import Twist, Pose, Pose2D
import GPSLib as gps


#Extended Kalman Filter. Assume linear sensor equation
class EKF(object):
	def __init__(self, C, Q, R, x0):
		self.f_cb = self.cb_f
		self.j_cb = self.cb_j
		self.C = np.mat(C)
		self.Q = np.mat(Q)
		self.R = np.mat(R)
		self.x = np.mat(x0)
		self.baseCoord = {'lat':0.0, 'lon':0.0}
		self.startHeading = 0.0
		self.yaw = 0.0
		self.use_gps_heading = True
		self.gps_heartbeat = 0
		self.speed = 0.0
		self.angularRate = 0.0
		self.starting = True
		self.stamp = rospy.Time()

		rows,cols = self.Q.shape
		self.m_size = rows
		self.P = np.eye(self.m_size)*10
		self.Fk = np.eye(self.m_size)		
		self.eul = None
		self.nav_sat = NavSatFix()

	def init3x3(self):
		self.f_cb = self.cb_f3
		self.j_cb = self.cb_j3
		self.C = [[1, 0, 0],[0, 1, 0], [0, 0, 1]]
		PNV = 1.0
		self.Q = np.eye(3)*PNV
		self.Q[2,2] = 0.2
		MNV = 0.02**2
		self.R = np.eye(3)*MNV
		self.x = np.mat([[0.0], [0.0],[self.yaw]])
		self.m_size = 3


	#Run iteration of extended kalman filter using the input signal and measurements
	def filter(self,u,y):
		Xpred = self.f_cb(self.x, u)
		
		self.Fk = self.j_cb(self.Fk, self.x, u)

		self.P = self.Fk*self.P*self.Fk.transpose() + self.Q

		innov = y - self.C*Xpred
		S = self.C*self.P*self.C.transpose() + self.R

		K = np.zeros((self.m_size,self.m_size))

		try:
			K = (self.P*self.C.transpose())*np.linalg.inv(S)
		except np.linalg.LinAlgError:
			K = np.zeros((self.m_size,self.m_size))

		self.x = Xpred + K*innov

		self.P = (np.eye(self.m_size) - K*self.C)*self.P
	
	
	#Imu message callback			
	def imu_cb(self,data):
		q = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
		self.eul = tf.transformations.euler_from_quaternion(q)
		rospy.loginfo(rospy.get_caller_id() + "Yaw is "  + str(math.degrees(self.eul[2])))
	
	#GPS message callback
	def gps_cb(self,data):
		self.nav_sat = data
		if self.nav_sat.status.status == NavSatStatus.STATUS_GBAS_FIX:
			rospy.loginfo("RTK-Lock")
		
	#Base GPS coordinate message callback
	def base_gps_cb(self,data):
		self.baseCoord['lat'], self.baseCoord['lon'] = data.position.x, data.position.y
		print "Base:", self.baseCoord['lat'], self.baseCoord['lon']
		q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
		self.yaw = math.degrees(tf.transformations.euler_from_quaternion(q)[2])
		self.startHeading = self.yaw
		print "Base Yaw:", self.yaw
		self.starting = False

	#velocity commands callback.
	def enc_cb(self,data):
		self.speed = data.linear.x
		self.angularRate = data.angular.z
	
	def gps_heading_cb(self,data):
		if self.use_gps_heading:
			self.yaw = data.theta
			self.gps_heartbeat+=1
			
	#Sets the relative base GPS coordinate to convert latitudes and longitudes
	#To positions in metres from base coordinate
	def setBaseCoord(self, lat, lon):
		self.baseCoord['lat'], self.baseCoord['lon'] = lat, lon
		self.starting = False
		
	def cb_f(self, x, u):#2x2 filter callback function
		x_new = x[0,0] + u[0,0]*math.sin(math.radians(u[1,0]))
		y_new = x[1,0] + u[0,0]*math.cos(math.radians(u[1,0]))
		return np.mat([[x_new], [y_new]])

	def cb_j(self, Fk, x, u): #2x2 filter Jacobian callback
		Fk[0,0], Fk[0,1] = 1.0, 0.0
		Fk[1,0], Fk[1,1] = 0.0, 1.0
		return Fk
		
	def cb_f3(self, x, u): #3x3 filter callback function
		x_new = x[0,0] + u[0,0]*math.sin(x[2,0])
		y_new = x[1,0] + u[0,0]*math.cos(x[2,0])
		theta_new = x[2,0] + u[1,0]
		return np.mat([[x_new],[y_new],[theta_new]])
		
	def cb_j3(self, Fk, x, u): #3x3 filter Jacobian callback
		Fk[0,0],Fk[0,1],Fk[0,2] = 1.0,0.0,u[0,0]*math.cos(x[2,0])
		Fk[1,0],Fk[1,1],Fk[1,2] = 0.0,1.0,-u[0,0]*math.sin(x[2,0])
		Fk[2,0],Fk[2,1],Fk[2,2] = 0.0,0.0,1.0
		return Fk
		
def main():
	C = [[1, 0],[0, 1]]
	
	x0 = [[0.0], [0.0]]
	PNV = 1.0
	Q = np.eye(2)*PNV

	MNV = 3.0**2
	R = np.eye(2)*MNV
	
	filter_heading = True
	if rospy.has_param("~filter_heading"):
		filter_heading = rospy.get_param("~filter_heading")
		
	ekf = EKF(C, Q, R, x0)
	if filter_heading:
		ekf.init3x3()
		
	pub = rospy.Publisher('kf_pose', Pose2D, queue_size=10)
	rospy.init_node('kf_node', anonymous=True)
	
	rospy.Subscriber("imu", Imu, ekf.imu_cb)
	rospy.Subscriber("gps", NavSatFix, ekf.gps_cb)
	rospy.Subscriber("base_gps", Pose, ekf.base_gps_cb)
	rospy.Subscriber("gps_heading", Pose2D, ekf.gps_heading_cb)
	rospy.Subscriber("pt_cmd",Twist, ekf.enc_cb)
	
	use_external_heading = False
	if rospy.has_param("~use_gps_heading"):
		ekf.use_gps_heading = rospy.get_param("~use_gps_heading")

	
	rate = rospy.Rate(10)
	kfTime = 0.0
	gps_heartbeat = ekf.gps_heartbeat
	theta_covariance = 0.0
	
	while not rospy.is_shutdown():
		gpsMsg = ekf.nav_sat
		curTime = rospy.get_rostime()
		
		gpsTime = gpsMsg.header.stamp.secs + gpsMsg.header.stamp.nsecs/1e9
		
		if not ekf.starting and gpsTime > kfTime:
			kfTime = gpsTime
			#Calculate the elapsed time
			deltaTime = (curTime.secs - ekf.stamp.secs) + (curTime.nsecs - ekf.stamp.nsecs)/1e9
			ekf.stamp.secs = curTime.secs
			ekf.stamp.nsecs = curTime.nsecs
			
			#Set sample time to 100 ms if expected value is much higher or 
			#Lower than expected
			if deltaTime < 0.0 or deltaTime > 1.0:
				print "Sample time wrong"
				deltaTime = 0.1
			
			#Convert GPS coordinate to coordinate in metres and filter
			#Using sent velocity/turn commands
			gpsCoord = {'lat':gpsMsg.latitude, 'lon':gpsMsg.longitude}
			coord = gps.getDistCoord(gpsCoord,ekf.baseCoord,0.0)
			angularVelocity = ekf.angularRate
			if use_external_heading:
				ekf.yaw = gps.constrainAngle(math.degrees(ekf.eul[2]))
				angularVelocity = 0.0
			if not filter_heading:
				ekf.yaw = gps.constrainAngle(ekf.yaw + math.degrees(angularVelocity)*deltaTime)
				
				u = np.mat([[deltaTime*ekf.speed], [ekf.yaw]])
				y = np.mat([[coord['x']],[coord['y']]])
			
				#Set the measurement noise variance using DOP measurements
				ekf.R = np.eye(2)*gpsMsg.position_covariance[0]
			else:
				u = np.mat([[deltaTime*ekf.speed],[deltaTime*ekf.angularRate]])
				y = np.mat([[coord['x']],[coord['y']],[math.radians(ekf.yaw)]])
				#Set the measurement noise variance using DOP measurements
				ekf.R = np.eye(3)*gpsMsg.position_covariance[0]
				if ekf.gps_heartbeat == gps_heartbeat:
					theta_covariance+=(deltaTime*ekf.angularRate)
				else:
					gps_heartbeat = ekf.gps_heartbeat
					theta_covariance = 0.0
				ekf.R[2, 2] = (gpsMsg.position_covariance[5] + theta_covariance) ** 2.0
				
			ekf.filter(u,y)
			
			#Convert back to GPS coordinates and publish
			coord['x'], coord['y'] = ekf.x[0,0], ekf.x[1,0]
			gpsCoord = gps.getGPSCoord(coord, ekf.baseCoord,0.0)
			msg = Pose2D()
			
			msg.x, msg.y = gpsCoord['lat'], gpsCoord['lon']
			msg.theta = ekf.yaw
			if filter_heading:
				msg.theta = math.degrees(ekf.x[2,0])
				
			log_data = str(msg.x) + ", " + str(msg.y) + ", " + str(msg.theta)
			rospy.loginfo(log_data)
			#print msg.x, msg.y
			pub.publish(msg)
		
			
		rate.sleep()
		
if __name__ == "__main__":
	main()
