#!/usr/bin/env python

import rospy
import roslib
import math
import tf
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

class AimingVelocity():
    '''
    aims the robot
    '''
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist)
    def send_vel(self, msg):
        '''
        Calculates which velocity should be sent to the base of the robot
        based on ar tag pose
        '''
        if len(msg.markers) == 0:
            print "empty"
            return
        y_relative = msg.markers[0].pose.pose.position.y
        twist_msg = Twist()
        if y_relative > 0:
            twist_msg.angular.z = 0.5
            print "To the left"
        elif y_relative < 0:
            twist_msg.angular.z = -0.5
            print "to the right"
        else:
            print "Dead center"
        self.pub.publish(twist_msg)
        
if __name__ == '__main__':
    aimer = AimingVelocity()
    rospy.init_node('ar_tag_angle_tracker')
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=aimer.send_vel)
    
    rospy.spin()

    
