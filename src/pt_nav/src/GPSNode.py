#!/usr/bin/env python
#GPS Node
#@file GPSNode.py

#@author Rinoy Pazhekattu (rinoy@rinoyp.com)
#@par Creation Date: 2016-08-01

#@par Copyrights:
#Copyright (c) 2016, Rinoy Pazhekattu.
#All rights reserved.

#Serial communication class for the Swift Binary Protocol
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import *
from sbp.navigation import *
from sbp.system import SBP_MSG_HEARTBEAT, MsgHeartbeat
import rospy
import math
import GPSLib as gps
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Twist, Pose, Pose2D


class SBPComm(object):
	tow = 0 #GPS time of week43.5330611796, -80.2193396999
	rtk_tow = 0 #RTK GPS time of week
	dop = {"gdop":0.0, "pdop":0.0, "tdop":0.0, "vdop":0.0, "hdop":0.0} #Dilution of Precision measurements
	n_sats = 0 #Number of satellites used for GPS measurement
	status = -1 #Status flags. 
	vn = 0.0   #North velocity in m/s
	ve = 0.0   #East velocity in m/s
	vd = 0.0   #Down velocity in m/s
	speed = 0.0
	base_lat, base_lon = 0.0, 0.0 #Base station Latitude and Longitude
	lat, lon = 0.0, 0.0 #GPS measured Latitude and Longitude
	height = 0.0 #GPS measured altitude
	heartbeat = 0 #SBP heartbeat status flags
	_driver = None
	_handler = None
	heading = 0.0 #GPS measured heading from base station to rover
	baseline = {'n':0.0, 'e':0.0, 'd':0.0} #North, East and Vertical Distance from rover to baseline in metres
	port = "/dev/serial/by-id/usb-FTDI_Single_RS232-HS-if00-port0"

	#Begin processing SBP messages on a background thread
	@staticmethod
	def begin():
		SBPComm._driver = PySerialDriver(SBPComm.port, baud=1000000)
		SBPComm._handler = Handler(Framer(SBPComm._driver.read, None, verbose = False))
		msg_types = [SBP_MSG_POS_LLH, SBP_MSG_VEL_NED, SBP_MSG_DOPS,SBP_MSG_BASELINE_NED,SBP_MSG_BASELINE_HEADING,SBP_MSG_HEARTBEAT]
		SBPComm._handler.add_callback(SBPComm.sbp_cb,msg_types)
		SBPComm._handler.start()

	#Callback to process message types registered for and get GPS data
	@staticmethod
	def sbp_cb(msg, **metadata):
		if msg.msg_type == SBP_MSG_POS_LLH:
			SBPComm.lat, SBPComm.lon = msg.lat, msg.lon
			SBPComm.height = msg.height
			SBPComm.n_sats = msg.n_sats
			SBPComm.tow = msg.tow
			SBPComm.status = msg.flags
		elif msg.msg_type == SBP_MSG_VEL_NED:
			SBPComm.vn = msg.n/1000.0
			SBPComm.ve = msg.e/1000.0
		elif msg.msg_type == SBP_MSG_DOPS:
			SBPComm.dop['gdop'], SBPComm.dop['pdop'], SBPComm.dop['tdop'] = msg.gdop/100.0, msg.pdop/100.0, msg.tdop/100.0
			SBPComm.dop['vdop'], SBPComm.dop['hdop'] = msg.vdop/100.0, msg.hdop/100.0
		elif msg.msg_type == SBP_MSG_BASELINE_HEADING:
			SBPComm.heading = msg.heading
		elif msg.msg_type == SBP_MSG_BASELINE_NED:
			SBPComm.baseline['n'], SBPComm.baseline['e'], SBPComm.baseline['d'] = msg.n/1000.0, msg.e/1000.0, msg.d/1000.0
			SBPComm.rtk_tow = msg.tow
		elif msg.msg_type == SBP_MSG_HEARTBEAT:
			SBPComm.heartbeat+=1
			
	#Parse GPS coordinates from handler only
	@staticmethod
	def simple_read():
		with PySerialDriver(USB_Piksi, baud=1000000) as driver:
			with Handler(Framer(driver.read, None, verbose=False)) as source:		
				for msg, metadata in source.filter(SBP_MSG_POS_LLH):
					# Store the latitude, longitude and number of satellites
					#print "Lat:", msg.lat, "Lon:", msg.lon, "nSats:", msg.n_sats
					
					SBPComm.lat, SBPComm.lon = msg.lat, msg.lon
					SBPComm.height = msg.height
					SBPComm.n_sats = msg.n_sats
					
	@staticmethod
	def pt_cb(data):
		SBPComm.speed = data.linear.x

def main():
	tow = 0.0
	pub1 = rospy.Publisher("gps",NavSatFix,queue_size=10)
	pub2 = rospy.Publisher("baseline",Pose,queue_size=10)
	pub3 = rospy.Publisher("gps_vel",Twist,queue_size=10)
	pub4 = rospy.Publisher("gps_heading",Pose2D,queue_size=10)
	rospy.init_node("gps_node",anonymous=False)
	rospy.Subscriber("pt_cmd",Twist, SBPComm.pt_cb)
	rate = rospy.Rate(20)
	h_cnt = 0
	last_north, last_east = None, None
	lastCoord = None
	
	heading_sample_t = 1000
	if rospy.has_param("~heading_sample_t"):
		heading_sample_t = rospy.get_param("~heading_sample_t")
	heading_sample_t/=100	
	heartbeat = SBPComm.heartbeat
	timeout_cnt = 0
	disconnected = True
	while not rospy.is_shutdown():
		msg = NavSatFix()
		if SBPComm.heartbeat != heartbeat:
			heartbeat = SBPComm.heartbeat
			timeout_cnt = 0
		elif timeout_cnt > 25:
			disconnected = True
		else:
			timeout_cnt+=1
			
		if tow != SBPComm.tow:
			disconnected = False
			msg.header.stamp = rospy.get_rostime()
			msg.latitude, msg.longitude = SBPComm.lat, SBPComm.lon
			msg.altitude = SBPComm.height
			
			baseError = 3.0
			msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
			if SBPComm.status == 0:
				msg.status.status = NavSatStatus.STATUS_FIX
				
				if h_cnt >= heading_sample_t:
					h_cnt = 0
					h_msg = Pose2D()
					curCoord = {'lat':msg.latitude,'lon':msg.longitude}
					h_msg.theta = gps.calcBearing(lastCoord,curCoord)
					pub4.publish(h_msg)
					s = "Heading: " + str(h_msg.theta)
					rospy.loginfo(s)
					lastCoord = {'lat':msg.latitude,'lon':msg.longitude}
				elif lastCoord is None:
					lastCoord = {'lat':msg.latitude, 'lon':msg.longitude}

			elif SBPComm.status >= 1:
				msg.status.status = NavSatStatus.STATUS_GBAS_FIX
				baseError = 0.02
				if SBPComm.status == 2:
					baseError = 2.0
					msg.status.status = NavSatStatus.STATUS_FIX
					
				if h_cnt >= heading_sample_t:
					h_cnt = 0
					h_msg = Pose2D()
					delta_north = SBPComm.baseline['n'] - last_north
					delta_east = SBPComm.baseline['e'] - last_east
					h_msg.theta = math.degrees(math.atan2(delta_east,delta_north))
					pub4.publish(h_msg)
					s = "Heading: " + str(h_msg.theta)
					rospy.loginfo(s)
					last_north = SBPComm.baseline['n']
					last_east = SBPComm.baseline['e']
					
				elif last_north is None:
					last_north = SBPComm.baseline['n']
					last_east = SBPComm.baseline['e']
				
				
				
			if SBPComm.speed > 0.0:
				h_cnt+=1
				
			bl_msg = Pose()
			bl_msg.position.x, bl_msg.position.y, bl_msg.position.z = SBPComm.baseline['n'], SBPComm.baseline['e'], SBPComm.baseline['d']
			pub2.publish(bl_msg)
			
			v_msg = Twist()
			v_msg.linear.x, v_msg.linear.y, v_msg.linear.z = SBPComm.vn, SBPComm.ve, SBPComm.vd
			pub3.publish(v_msg)
			
			for i in range(0,6,4):
				msg.position_covariance[i] = (SBPComm.dop['hdop']*baseError)**2.0
				
			msg.position_covariance[8] = (SBPComm.dop['vdop']*baseError)**2.0
			heading_period = heading_sample_t/10.0
			speed = math.sqrt(SBPComm.vn**2.0 + SBPComm.ve**2.0)
			msg.position_covariance[5] = math.atan2(2.0*(SBPComm['hdop']*baseError),speed*(heading_sample_t/10.0))
			msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_GALILEO | NavSatStatus.SERVICE_COMPASS
			s = str(msg.latitude) + ", " + str(msg.longitude)
			rospy.loginfo(s)
			pub1.publish(msg)
		elif disconnected:
			msg.status.status = NavSatStatus.STATUS_NO_FIX
			pub1.publish(msg)
		
				
		tow = SBPComm.tow


		rate.sleep()	
		
if __name__ == "__main__":
	if rospy.has_param("~piksi_port"):
		SBPComm.port = rospy.get_param("~piksi_port")
	SBPComm.begin()
	try:
		main()
	except rospy.ROSInterruptException:
		pass
