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
        self.pub = rospy.Publisher("/rosaria/cmd_vel", Twist, queue_size=10)
    def send_vel(self, msg):
        '''
        Calculates which velocity should be sent to the base of the robot
        based on ar tag pose
        '''
        if len(msg.markers) == 0:
            print "empty"
            return
        x_relative = msg.markers[0].pose.pose.position.x
        y_relative = msg.markers[0].pose.pose.position.y
        angle = math.atan(abs(y_relative)/abs(x_relative))
        print "ANGLE: ", angle
        if angle > 0.2:
            vel_mag = 0.2
        elif angle > 0.05:
            vel_mag = 0.1
        else:
            vel_mag = 0
        twist_msg = Twist()
        if y_relative > 0:
            twist_msg.angular.z = vel_mag
            print "To the left"
        elif y_relative < 0:
            twist_msg.angular.z = -vel_mag
            print "to the right"
        else:
            print "Dead center"
        self.pub.publish(twist_msg)
        
if __name__ == '__main__':
    aimer = AimingVelocity()
    rospy.init_node('ar_tag_angle_tracker')
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=aimer.send_vel)
    
    rospy.spin()

    
