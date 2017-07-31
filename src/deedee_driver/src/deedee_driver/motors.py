#! /usr/bin/env python

import rospy
from rospy import Subscriber, Publisher
from robot_control.msg import WheelSpeeds
from std_msgs.msg import Float64

class MotorsDriverSim:
  def __init__(self):
    rospy.init_node("motors_driver_sim")
    rospy.sleep(0.5)

    self.wheel_speeds_sub = Subscriber("/wheel_speeds",
                                       WheelSpeeds,
                                       self.on_wheel_speeds)


    self.l_wheel_cmd_pub = Publisher("/deedee_left_wheel_controller/command",
                                     Float64,
                                     queue_size=10)
    self.r_wheel_cmd_pub = Publisher("/deedee_right_wheel_controller/command",
                                     Float64,
                                     queue_size=10)

    rospy.spin()

  def on_wheel_speeds(self, speeds):
    self.l_wheel_cmd_pub.publish(float(speeds.left))
    self.r_wheel_cmd_pub.publish(float(speeds.right))

