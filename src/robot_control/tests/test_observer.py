#! /usr/bin/env python

PKG='robot_control'

import sys
import unittest
from robot_control.observer_lib import *

class TestObserver(unittest.TestCase):
  ''' Unit tests for the Observer class
  '''

  def test_no_estimate_if_no_reading(self):
    ''' Test that the observer gives no estimate if no reading ever came in
    '''
    obs = IdealObs()
    sensor_readings = {"camera": 10.,
                       "lidar": 43.,
                       "sonar": 2.}
    state = obs.estimate(sensor_readings)

    self.assertIsNone(state,
                      "Expected the state to be None, got %s" % state)


if __name__ == '__main__':
  import rosunit

  rosunit.unitrun(PKG, 'test_Observer', TestObserver)

