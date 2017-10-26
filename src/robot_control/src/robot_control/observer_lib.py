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


class IdealObs:
  ''' Definition of an ideal observer

      Detail:
        This controller's estimate matches the actual state perfectly
        Each sensor status is described by True (active) or False (inactive)
  '''
  def __init__(self):
    self.state = None

    self.sensor_list = ["true_state"]

    self.sensor_config = {"true_state": (True, 2.)} # (required, timeout)

    self.active_flag = {"true_state": False}

  def check_readings(self, sensor_readings, t):
    ''' Check the validity of the sensor data
    '''
    # Check for sensor timeout
    for k in sensor_readings.keys():
      if (t - sensor_readings[k][1]
          > self.sensor_config[k][1]):
        self.active_flag[k] = False
      else:
        self.active_flag[k] = True

    # Check that all required sensors are active
    green_light = True
    for k in self.sensor_list:
      green_light = green_light and (self.active_flag[k]
                                     or not self.sensor_config[k][0])

    return green_light

  def estimate(self, sensor_readings):
    ''' Estimation of the robot state

        Detail:
          The Gazebo model state is converted to retrain only the 2D pose
          of the mobile robot
    '''
    state = sensor_readings["true_state"]

    self.state = state
    return state

