#!/usr/bin/env python

#Navigation Server
#@file NavServer.py

#@author Rinoy Pazhekattu (rinoy@rinoyp.com)
#@par Creation Date: 2016-08-01

#@par Copyrights:
#Copyright (c) 2016, Rinoy Pazhekattu.
#All rights reserved.

import rospy
import math 
import threading
import actionlib, tf
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Twist, Pose, Pose2D, Quaternion
import GPSLib as gps

class PurePursuit(threading.Thread):
	TRACKER_STATUS_WAITING,TRACKER_STATUS_ACTIVE = 0,1

	def __init__(self, path=None, lookahead_distance=4.5, desired_speed = 1.2, maxAngularRate = 1.0, loops = 1):
		threading.Thread.__init__(self)
		self.heading = 0.0
		self.pose = {'lat':0.0, 'lon':0.0}
		self.path = path
		self.loops = loops
		self.finishedLoop = False
		self.trackedPoint = 0
		self.lookahead_distance = lookahead_distance
		self.distanceTravelled = 0.0
		if desired_speed > 0.0:
			self.desired_speed = desired_speed
		else:
			self.desired_speed = 1.5

		self.max_angular_rate = maxAngularRate
		self.tf_listener = tf.TransformListener()
		self.trans, self.rot = 0.0, 0.0
		self.preempted = False
		self.failed = False

		self.ppc_time = 0.0
		self.kf_ts = rospy.get_rostime()
		self.cmd = Twist()
		self.stopping = False
		self.tracking = False
		self.status = self.TRACKER_STATUS_WAITING
		self.setDaemon(True)
		self.start()
		
	def run(self):
		while not self.stopping:
			if rospy.is_shutdown(): #Stop if ROS is shutdown
				self.stopping = True
			if not self.tracking:
				continue
					
			
			kf_time = self.kf_ts.secs + self.kf_ts.nsecs/1e9
			if kf_time > self.ppc_time: #Wait for a pose measurement
				self.ppc_time = kf_time
			else:
				continue
			
			
			
			if self.preempted: #Stop if client preempts goal
				self.cmd.linear.x = 0.0
				self.cmd.angular.z = 0.0
				self.tracking = False

			try:
				(trans,rot) = self.tf_listener.lookupTransform('/piksi','/base',rospy.Time.now())
				self.trans = math.sqrt(trans[0]**2.0 + trans[1]**2.0)
				self.rot = gps.constrainAngle(self.heading + math.degrees(tf.transformations.euler_from_quaternion(rot)[2]))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				self.trans, self.rot = 0.0, 0.0

			self.trackedPoint = self.findClosestPoint()

			#Starting from closest point, move up path and find the goal point
			goalPoint = self.findGoalPoint()	
			if goalPoint is None: #No goalpoint found, finished
				
				self.cmd.linear.x = 0.0
				self.cmd.angular.z = 0.0
				
				self.tracking = False
				stopDistance = gps.getDistance(self.pose,self.path[self.trackedPoint])
				if stopDistance > 5.0:
					self.failed = True
			
			else:
				#Tranform the goal point into vehicle coordinates
				vehicleCoord = gps.getDistCoord(goalPoint,self.pose,self.heading)
					
				#Caclulate the angular velocity to send
				self.cmd.linear.x = self.desired_speed
				
				curvature = self.desired_speed*2.0*vehicleCoord['x']/self.lookahead_distance**2.0
				
				s =  "Goal Point:" + str(goalPoint['lat']) + "," + str(goalPoint['lon']) + "\n"
				rospy.loginfo(s)
				#Constrain values to stay less than the max angular rate
				if curvature >= 0.0:
					self.cmd.angular.z = min(self.max_angular_rate, curvature)
				elif curvature < 0.0:
					self.cmd.angular.z = max(-self.max_angular_rate,curvature)
				
				 
	def findClosestPoint(self):
		curPose = gps.getCoord(self.pose,self.trans,self.rot)
		distances = [gps.getDistance(curPose,coord) for coord in self.path]
		minDistance = distances[0]
		pointIndex = 0
		for i in range(1,len(distances)):
			if distances[i] < minDistance:
				minDistance = distances[i]
				pointIndex = i
				
		return pointIndex
	
	def findGoalPoint(self):
		found = False
		pointIndex = self.trackedPoint
		if pointIndex < len(self.path)-1:
			distanceToNext = gps.getDistance(self.path[pointIndex],self.path[pointIndex+1])
		else:
			distanceToNext = gps.getDistance(self.path[pointIndex], self.path[0])

		curPoint = self.path[pointIndex]
		curHeading = self.path[pointIndex]['heading']
		distance = 0.0
		goalPoint = None
		spacing = 0.001
		curPose = gps.getCoord(self.pose,self.trans,self.rot)
		
		closestPoint = None
		closestDistance = -1.0
		if self.finishedLoop:
			if self.loops > 1 and self.trackedPoint == 0: ##Stop when closest point is first point if looped path
				self.finishedLoop = False
				return None
			elif self.trackedPoint == len(self.path)-1: #Stop when closest to last point if not a looped path
				self.finishedLoop = False
				return None

		while not found:
			goalPoint = gps.getCoord(curPoint,spacing,curHeading)
			goalDistance = gps.getDistance(curPose,goalPoint)
			#Found if within one centimetre of lookahead distance
			if closestPoint is None:
				closestPoint = goalPoint
				closestDistance = goalDistance
			elif closestDistance > goalDistance:
				closestDistance = goalDistance
				closestPoint = goalPoint
			
			if goalDistance >= self.lookahead_distance-0.1 and goalDistance <= self.lookahead_distance+0.1:
				found = True
			elif goalDistance > self.lookahead_distance+1.0:
				return closestPoint
				
			curPoint = goalPoint
			distance+=spacing
			if distance >= distanceToNext:
				if pointIndex < len(self.path)-1:
					pointIndex+=1
					
					distance = 0.0
					curPoint = self.path[pointIndex]
					curHeading = self.path[pointIndex]['heading']
					if pointIndex == len(self.path)-1:
						self.finishedLoop = True
						distanceToNext = gps.getDistance(curPoint, self.path[0])
					else:	
						distanceToNext = gps.getDistance(curPoint,self.path[pointIndex+1])
				elif pointIndex == len(self.path)-1:
					if self.loops > 1:
						pointIndex = 0
						distance = 0.0
						curPoint = self.path[pointIndex]
						curHeading = self.path[pointIndex]['heading']
						distanceToNext = gps.getDistance(curPoint, self.path[pointIndex + 1])
					else:
						return None
				else: #Reached the end of path. No lookahead point found
					return closestPoint
					
		return goalPoint

	def setPreempted(self):
		self.preempted = True
		
	def kf_cb(self,data):
		self.pose['lat'], self.pose['lon'] = data.x, data.y
		self.heading = data.theta
		self.kf_ts = rospy.get_rostime()
		
class NavigateAction(object):
	MAX_ANGULAR_RATE = math.pi/5.0
	TRACKER_STATUS_WAITING,TRACKER_STATUS_ACTIVE = 0,1

	baseCoord = None
	def __init__(self, name):
		self.pos = {'lat':0.0, 'lon':0.0}
		self.yaw = 0.0
		self.nav_sat = NavSatFix()
		self.rtk_timeout = 0
		self.actuator = rospy.Publisher("pt_cmd",Twist,queue_size=10)
		self.feedback = pt_actions.msg.NavigateFeedback()
		self.result = pt_actions.msg.NavigateResult()
		self.base = rospy.Publisher("base_gps",Pose,queue_size=10)
		self._action_name = name
		self.preempted = False
		self._as = actionlib.SimpleActionServer(self._action_name, pt_actions.msg.NavigateAction, execute_cb=self.follow_path, auto_start = False)
		self._as.register_preempt_callback(self.preempt_cb)
		self._as.start()
		self.PPC = PurePursuit()
		self.gps_ts = rospy.Time()
		self.externalControl = False
		if rospy.has_param("~auto_lookahead"):
			self.PPC.auto = rospy.get_param("~auto_lookahead")
	
	def findClosestPoint(self,goal):
		pose = {'lat':self.nav_sat.latitude,'lon':self.nav_sat.longitude}
		coord = {'lat':goal.lats[0],'lon':goal.lons[0]}
		minDistance = gps.getDistance(pose,coord)
		closestPoint = 0
		for i in range(1,len(goal.lats)-1):
			coord = {'lat':goal.lats[i],'lon':goal.lons[i]}
			distance = gps.getDistance(pose,coord)
			if distance < minDistance: 
				minDistance = distance
				closestPoint = i
		
		return closestPoint
		
	def follow_path(self, goal):
		rospy.loginfo("Starting path")
		success = True
		waiting = True
		r = rospy.Rate(1)

			
		#Wait for the GPS to get an RTK lock
		while waiting:
			if self.nav_sat.status.status == NavSatStatus.STATUS_GBAS_FIX or rospy.is_shutdown():
				waiting = False
				break
				
				
			r.sleep()
			
		#Initialize the Kalman Filter by finding closest point on path and sending it a base coordinate.
		baseCoord = Pose()			
		nCoords = len(goal.lats)
		baseCoord.position.x, baseCoord.position.y, baseCoord.position.z = goal.lats[0], goal.lons[0], self.nav_sat.altitude
		closestPoint = self.findClosestPoint(goal)
		print "Closest Point:", closestPoint
		coord1 = {'lat':goal.lats[closestPoint], 'lon':goal.lons[closestPoint]}
		coord2 = {'lat':goal.lats[closestPoint+1], 'lon':goal.lons[closestPoint+1]}
		print "Coord 1:", coord1['lat'], coord1['lon']
		print "Coord 2:", coord2['lat'], coord2['lon']
		startHeading = math.radians(gps.calcBearing(coord1,coord2))
		startQ = tf.transformations.quaternion_from_euler(0.0,0.0,startHeading)
		print "Base Heading:",math.degrees(startHeading)
		baseCoord.orientation.x, baseCoord.orientation.y = startQ[0], startQ[1]
		baseCoord.orientation.z, baseCoord.orientation.w = startQ[2], startQ[3]
		
		
		self.base.publish(baseCoord)
		NavigateAction.baseCoord = baseCoord
			
		path = []
		lastHeading = 0.0
		for i in range(len(goal.lats)):
			coord = {'lat':goal.lats[i], 'lon':goal.lons[i]}
			if i < len(goal.lats)-1:
				nextCoord = {'lat':goal.lats[i+1], 'lon':goal.lons[i+1]}
				coord['heading'] = gps.calcBearing(coord,nextCoord)
				lastHeading = coord['heading']
			else:
				if goal.loops < 2:
					coord['heading'] = lastHeading
				else:
					coord['heading'] = gps.calcBearing(coord,path[0]);

			path.append(coord)
			
		self.PPC.path = path
		self.PPC.lookahead_distance = goal.lookahead_distance
		self.PPC.desired_speed = goal.desired_speed
		self.PPC.loops = goal.loops
		self.PPC.trackedPoint = 0
		self.PPC.distanceTravelled = 0.0
		r = rospy.Rate(20)
		r.sleep()
		for i in range(goal.loops):
			self.PPC.tracking = True
			lastTime = rospy.get_rostime()
			while self.PPC.tracking:
				curTime = rospy.get_rostime()
				deltaTime = (curTime.secs - lastTime.secs) + (curTime.nsecs-lastTime.nsecs)/1e9
				#s =  "Period: " + str(deltaTime)
				#rospy.loginfo(s)
				lastTime.secs, lastTime.nsecs = curTime.secs, curTime.nsecs 
				if rospy.is_shutdown():
					break
				if self.rtk_timeout >= 10 or self.externalControl: #Only send commands when we have an RTK lock
					self.feedback.status = self.TRACKER_STATUS_WAITING
					self._as.publish_feedback(self.feedback)
					cmd = Twist()
					cmd.linear.x, cmd.angular.z = 0.0, 0.0
					self.actuator.publish(cmd)
					continue

				deltaGPSTime = (curTime.secs - self.gps_ts.secs) + (curTime.nsecs - self.gps_ts.nsecs)/1e9
				if deltaGPSTime > 1.0:#Stop if not receiving updates from GPS
					self.PPC.tracking = False
					self.PPC.failed = True
				self.PPC.distanceTravelled+=(self.PPC.cmd.linear.x*deltaTime)
				s = "Cmd: " + str(self.PPC.cmd.linear.x) + ", " + str(self.PPC.cmd.angular.z) + "\n"
				rospy.loginfo(s)
				self.actuator.publish(self.PPC.cmd)
				self.feedback.curPoint = self.PPC.trackedPoint
				self.feedback.latitude, self.feedback.longitude = self.PPC.pose['lat'], self.PPC.pose['lon']
				self.feedback.status = self.TRACKER_STATUS_ACTIVE
				self.feedback.distanceTravelled = self.PPC.distanceTravelled
				self._as.publish_feedback(self.feedback)
				r.sleep()

			self.PPC.trackedPoint = 0
			if self.preempted or rospy.is_shutdown() or self.PPC.failed:
				self.preempted = False
				self.PPC.preempted = False
				self.PPC.failed = False
				success = False
				self._as.set_aborted()
				break
				
		if success:
			rospy.loginfo('%s: Succeeded' % self._action_name)
			rospy.loginfo("Travelled " + str(self.PPC.distanceTravelled) + " metres")
			self.result.end_lat, self.result.end_lon = self.PPC.pose['lat'], self.PPC.pose['lon']
			self.result.end_distance = gps.getDistance(self.PPC.path[len(self.PPC.path)-1],self.PPC.pose)
			self._as.set_succeeded(self.result)
	
	def preempt_cb(self):
		self.preempted = True
		self.PPC.setPreempted()
			
	def kf_cb(self,data):
		self.pos['lat'], self.pos['lon'] = data.x, data.y
		self.yaw = data.theta
		self.PPC.kf_cb(data)
		#print str(self.pos['lat']) + ",",self.pos['lon']
		
	def gps_cb(self,data):
		self.nav_sat = data
		if self.nav_sat.status.status != NavSatStatus.STATUS_GBAS_FIX:
			self.rtk_timeout+=1
		else:
			self.rtk_timeout = 0
		self.gps_ts = rospy.get_rostime()
			
		
if __name__ == '__main__':
	rospy.init_node('ptServer')
	nav = NavigateAction(rospy.get_name())
	rospy.Subscriber("kf_pose", Pose2D, nav.kf_cb)
	rospy.Subscriber("gps",NavSatFix,nav.gps_cb)

	rospy.spin()
