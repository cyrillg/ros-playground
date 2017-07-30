#!/usr/bin/env python

import rospy
from robot_control.msg import WheelSpeeds

class Controller:
  def __init__(self):
    rospy.init_node("controller")
    rospy.sleep(0.5)

    self.wheel_speeds_pub = rospy.Publisher("/wheel_speeds",
                                            WheelSpeeds,
                                            queue_size=10)

    self.run()

  def run(self):
    rate = rospy.Rate(10) # 10hz
    t_start = rospy.Time.now().to_sec()

    msg = WheelSpeeds()
    t = rospy.Time.now().to_sec() - t_start
    while t<5.0:
      msg.left = 1.5
      msg.right = 1.5
      self.wheel_speeds_pub.publish(msg)
      rate.sleep()
      t = rospy.Time.now().to_sec() - t_start

    while t<10.0:
      msg.left = -1.5
      msg.right = 1.5
      self.wheel_speeds_pub.publish(msg)
      rate.sleep()
      t = rospy.Time.now().to_sec() - t_start

    msg.left = 0.0
    msg.right = 0.0
    self.wheel_speeds_pub.publish(msg)

if __name__=="__main__":
  Controller()
