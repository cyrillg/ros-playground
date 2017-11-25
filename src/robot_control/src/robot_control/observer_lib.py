'''
Definition of observer classes

Observers gather the methods used to compute the current estimate of the robot
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
          This controller's estimate matches the actual state perfectly.

          It follows the following generic template:
          - Each sensor status is described by True (active) or False (inactive).
          - They all start inactive, and are activated on reception of the first
            reading.
          - A sensor can become inactive again if no reading is received for
            longer than the timeout duration.
          - Sensors can be declared required or not, so that estimation stops
            or continues when one or more required sensors are inactive.
          - A "check_readings(self, sensor_readings, t)" method is required, used
            to verify the validity of the current batch of sensor readings.
            Timeout checking is the minimum, but can extend to other checks.
          - An "estimate(self, sensor_readings)" method is required, computing
            the actual estimate.
    '''
    def __init__(self, robot_name, config):
        # Configs
        true_state_required = config["true_state"]["required"]
        true_state_timeout = config["true_state"]["timeout"]

        self.state = None
        self.sensor_list = ["true_state"]
        self.sensor_config = {"true_state": (true_state_required,
                                             true_state_timeout)}
        self.active_flag = {"true_state": False}

    def check_readings(self, sensor_readings, t):
        ''' Check the validity of the sensor data

            Detail:
              Check for timeout in the given sensor readings. Return False if
              one or more of the required sensors are inactive.
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
              The robot pose is made equal to the Gazebo model's pose
        '''
        state = sensor_readings["true_state"][0]

        self.state = state
        return state

