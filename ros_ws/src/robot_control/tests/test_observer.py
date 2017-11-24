#! /usr/bin/env python

PKG='robot_control'

import sys
import unittest
from robot_control.observer_lib import *

class TestObserver(unittest.TestCase):
    ''' Unit tests for the Observer class
    '''

    def setUp(self):
        robot_name = "test_bot"
        config = {"true_state": {"required": True,"timeout": 2.}}
        self.obs = IdealObs(robot_name, config)

    def test_check_fails_if_no_reading(self):
        ''' Test that the observer gives no estimate if no reading ever came in
        '''
        sensor_readings = {}
        state = None
        green_light = self.obs.check_readings(sensor_readings,0)

        self.assertIsNone(state,
                          "Expected green_light to be False, got %s" % green_light)


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, 'test_Observer', TestObserver)

