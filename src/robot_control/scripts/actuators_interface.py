#!/usr/bin/env python

import rospy
from robot_control.msg import WheelSpeeds
from std_msgs.msg import Float64

class ActuatorsInterface:
  def __init__(self):
    rospy.init_node("actuators_interface")
    rospy.sleep(0.5)

    self.wheel_speeds_sub = rospy.Subscriber("/wheel_speeds",
                                           WheelSpeeds,
                                           self.on_wheel_speeds)

    self.left_wheel_cmd_pub = rospy.Publisher("/deedee_left_wheel_controller/command",
                                              Float64,
                                              queue_size=10)
    self.right_wheel_cmd_pub = rospy.Publisher("/deedee_right_wheel_controller/command",
                                               Float64,
                                               queue_size=10)

    rospy.spin()

  def on_wheel_speeds(self, speeds):
    self.left_wheel_cmd_pub.publish(float(speeds.left))
    self.right_wheel_cmd_pub.publish(float(speeds.right))

if __name__=="__main__":
  ActuatorsInterface()
