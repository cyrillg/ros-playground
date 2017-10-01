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

from tf.transformations import euler_from_quaternion as rpy_from_q
from geometry_msgs.msg import Pose2D

class IdealObs:
  ''' Definition of an ideal observer

      Detail:
        This controller's estimate matches the actual state perfectly
        Detail:
  '''
  def check_readings(self, sensor_readings):
    ''' TODO
    '''
    return ("true_state" in sensor_readings.keys()
            and sensor_readings["true_state"])

  def estimate(self, sensor_readings):
    ''' Estimation of the robot state

        Detail:
          The Gazebo model state is converted to retrain only the 2D pose
          of the mobile robot
    '''
    if self.check_readings(sensor_readings):
      state = sensor_readings["true_state"]
      idx = state.name.index("deedee")

      quaternion = (state.pose[idx].orientation.x,
                    state.pose[idx].orientation.y,
                    state.pose[idx].orientation.z,
                    state.pose[idx].orientation.w)
      rpy = rpy_from_q(quaternion)

      state = Pose2D(x=state.pose[idx].position.x,
                     y=state.pose[idx].position.y,
                     theta=rpy[2])
    else:
      state = None

    return state

