#! /usr/bin/env python
#Navigation Client
#@file NavClient.py

#@author Rinoy Pazhekattu (rinoy@rinoyp.com)
#@par Creation Date: 2016-08-01

#@par Copyrights:
#Copyright (c) 2016, Rinoy Pazhekattu.
#All rights reserved.

import rospy
 
import actionlib
import pt_actions.msg
import argparse, sys
import serial
import threading
from sensor_msgs.msg import NavSatFix, NavSatStatus
import GPSLib as gps

class NavClient(object):
	def __init__(self):
		self.nav_sat = NavSatFix()
		self.nav_sat.status.status = NavSatStatus.STATUS_NO_FIX
		rospy.Subscriber("gps",NavSatFix,self.gps_cb)
		self.client = actionlib.SimpleActionClient(ptServer',pt_actions.msg.NavigateAction)
		self.goal = pt_actions.msg.NavigateGoal()
		self.feedback = None
		
	def send_goal(self):
		self.client.wait_for_server()
		args = Args()
		if rospy.has_param("~threshold"):
			args.threshold = rospy.get_param("~threshold")
		if rospy.has_param("~lookahead_distance"):	
			args.lookahead_distance = rospy.get_param("~lookahead_distance")
		if rospy.has_param("~desired_speed"):
			args.desired_speed = rospy.get_param("~desired_speed")
		if rospy.has_param("~max_angular_velocity"):
			args.max_angular_speed = rospy.get_param("~max_angular_velocity")
		if rospy.has_param("~loops"):	
			args.loops = rospy.get_param("~loops")
		if rospy.has_param("~auto_gen_bounds"):	
			args.auto = rospy.get_param("~auto_gen_bounds")
		
		
		if args.auto:
			args.auto_bounds_size = rospy.get_param("~auto_bounds_size")
			args.auto_start_heading = rospy.get_param("~auto_start_heading")
			if rospy.has_param("~auto_shape"):
				args.auto_shape = rospy.get_param("~auto_shape")
			
		if rospy.has_param("~input_file"):
			args.input_file = open(rospy.get_param("~input_file"))
		if args.input_file is None and not args.auto:
			sys.exit()
		
		# Creates a goal to send to the action server.
		self.goal.threshold = args.threshold
		self.goal.loops = args.loops
		self.goal.lookahead_distance = args.lookahead_distance
		self.goal.desired_speed = args.desired_speed
		self.goal.max_angular_speed = args.max_angular_speed
		self.goal.start_point = 0
		info = "Sending goal to NavServer"
		rospy.loginfo(info)	
		
		if args.auto:
			while self.nav_sat.status.status == NavSatStatus.STATUS_NO_FIX:
				pass
			c = {'lat':self.nav_sat.latitude, 'lon':self.nav_sat.longitude}
			coords = None
			if "circle" in args.auto_shape.lower():
				coords = gps.drawCircle(c,args.auto_bounds_size,args.auto_start_heading)
			elif "line" in args.auto_shape.lower():
				coords = gps.drawLine(c,args.auto_bounds_size,args.auto_start_heading)
			else:
				coords = gps.drawSquare(c,args.auto_bounds_size,args.auto_start_heading)
			self.goal.lats, self.goal.lons = [coords[i]['lat'] for i in range(len(coords))], [coords[i]['lon'] for i in range(len(coords))]
					
		else:
			for line in args.input_file:
				toks = line.split(",")
				if len(toks) >= 2:
					lat = float(toks[0])
					lon = float(toks[1])
				
					self.goal.lats.append(lat)
					self.goal.lons.append(lon)
					print lat, lon
					
		# Waits for the server to finish performing the action.
		stopping = False
		r = rospy.Rate(5)
		
		# Sends the goal to the action server.
		self.client.send_goal(goal=self.goal,feedback_cb=self.feedback_cb)
		
		timeout_cnt = 0
		
		print "Sent goal to server..."
		while not stopping:
			state = self.client.get_state()
			if state == actionlib.GoalStatus.SUCCEEDED:
				print "Goal succeeded"
				stopping = True
			if rospy.is_shutdown():
				print "Shutting Down"
				stopping = True
			r.sleep()
			
		return self.client.get_result()
		
	def feedback_cb(self,feedback):
		self.feedback = feedback
		
	def gps_cb(self, data):
		self.nav_sat = data
		
class Args(object):	
	def __init__(self):
		self.input_file = None
		self.threshold = 1.0
		self.auto = False
		self.auto_bounds_size = 15.0
		self.auto_start_heading = 0.0
		self.loops = 1
		self.lookahead_distance = 4.5
		self.desired_speed = 1.2
		self.max_angular_speed = 1.0
		self.compass = False
		self.auto_shape = "square"

if __name__ == '__main__':
	try:
		rospy.init_node('ptClient')
		nav_client = NavClient()
		result = nav_client.send_goal()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
