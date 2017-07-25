#!/usr/bin/env python

import rospy
from robot_control.msg import WheelSpeeds

class Controller:
  def __init__(self):
    rospy.init_node("actuators_interface")
    rospy.sleep(0.5)

    self.wheel_speeds_pub = rospy.Publisher("/wheel_speeds",
                                            WheelSpeeds,
                                            queue_size=10)

    self.run()

  def run(self):
    rate = rospy.Rate(10) # 10hz
    t_start = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():

      t = rospy.Time.now().to_sec()
      while t<5.0:
        t = rospy.Time.now().to_sec() - t_start
        msg = WheelSpeeds()
        msg.left = 1.5
        msg.right = 1.5
        self.wheel_speeds_pub.publish(msg)
        rate.sleep()

      while t<10.0:
        t = rospy.Time.now().to_sec() - t_start
        msg = WheelSpeeds()
        msg.left = -1.5
        msg.right = 1.5
        self.wheel_speeds_pub.publish(msg)
        rate.sleep()

      msg = WheelSpeeds()
      msg.left = 0.0
      msg.right = 0.0
      self.wheel_speeds_pub.publish(msg)
      rate.sleep()

if __name__=="__main__":
  Controller()
