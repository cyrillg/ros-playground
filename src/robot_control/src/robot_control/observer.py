'''
Definition of observer classes

Observers gather the functions used to compute the current estimate of the
state

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

import rospy
from robot_control.utils import *
from rospy import Subscriber, Publisher
from tf.transformations import euler_from_quaternion as rpy_from_q

from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelStates

class IdealObs:
    ''' Definition of an ideal observer

        Detail:
          This controller's estimate matches the actual state perfectly
    '''
    def __init__(self):
      rospy.init_node("observer")
      rospy.sleep(0.5)

      self.pose_sub = Subscriber("/gazebo/model_states",
                                 ModelStates,
                                 self.on_model_states)
      self.state_pub = Publisher("state",
                                 Pose2D,
                                 queue_size=10)

      rospy.spin()

    def on_model_states(self, state):
      ''' Gazebo model state callback

          Detail:
            The Gazebo model state is converted to retrain only the 2D pose
            of the mobile robot
      '''
      idx = state.name.index("deedee")

      quaternion = (state.pose[idx].orientation.x,
                    state.pose[idx].orientation.y,
                    state.pose[idx].orientation.z,
                    state.pose[idx].orientation.w)
      rpy = rpy_from_q(quaternion)

      state = Pose2D(x=state.pose[idx].position.x,
                     y=state.pose[idx].position.y,
                     theta=rpy[2])

      self.state_pub.publish(state)

