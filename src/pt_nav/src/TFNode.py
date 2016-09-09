#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Pose2D

class Transformer(object):
    #Initializes the transformer. Set piksi_x and piksi_y variables to X and Y offsets
    #from centre of the robot
    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        self.piksi_x, self.piksi_y, self.piksi_z = x, y, z
        self.heading = 0.0
        self.br = tf.TransformBroadcaster()


    def heading_cb(self,data):
        self.heading = data.theta
        dist = math.sqrt(self.piksi_x**2.0 + self.piksi_y**2.0)
        x, y = dist*math.sin(math.radians(self.heading)), dist*math.cos(math.radians(self.heading))
        h = math.atan2(self.piksi_x,self.piksi_y)
        q = tf.transformations.quaternion_from_euler(0.0,0.0,h)
        self.br.sendTransform((x,y,0.0),q,rospy.Time.now(),"piksi","base")

if __name__ == "__main__":
    try:
        tForm = Transformer()
        rospy.init_node("TFNode")
        piksi_x, piksi_y, piksi_z = [0.0 for i in range(3)]
        if rospy.has_param("~piksi_x"):
            piksi_x = rospy.get_param("~piksi_x")
        if rospy.has_param("~piksi_y"):
            piksi_y = rospy.get_param("~piksi_y")
        if rospy.has_param("~piksi_z"):
            piksi_z = rospy.get_param("~piksi_z")
        tForm = Transformer(piksi_x,piksi_y,piksi_z)
        rospy.Subscriber("gps_heading", Pose2D, tForm.heading_cb)
        rospy.spin()
    except rospy.ROSInternalException:
        pass
