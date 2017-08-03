#!/usr/bin/env python

import rospy
from rospy import Subscriber, Publisher
from robot_control.msg import WheelSpeeds
from geometry_msgs.msg import Twist

class DiffDrive:
  def __init__(self):
    rospy.init_node("diff_drive_model")
    rospy.sleep(0.5)

    self.L = 0.6
    self.r = 0.15

    self.wheel_speeds_sub = Subscriber("twist_cmd",
                                       Twist,
                                       self.on_twist)

    self.wheel_speeds_pub = Publisher("wheel_speeds_cmd",
                                      WheelSpeeds,
                                      queue_size=10)

    rospy.spin()

  def on_twist(self, vel):
    print "New velocity!"
    (u0, u1) = self.transform(vel.linear.x, vel.angular.z)

    msg = WheelSpeeds()
    msg.left = u1
    msg.right = u0
    self.wheel_speeds_pub.publish(msg)

  def transform(self, v, w):
    ''' Transform linear and angular speed into wheel angular speeds
    '''
    u0 = (2*v + self.L*w) / (2*self.r)
    u1 = (2*v - self.L*w) / (2*self.r)

    return (u0, u1)

