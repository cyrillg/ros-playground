#! /usr/bin/env python

PKG='deedee_driver'

import sys
import unittest
from mock import Mock

from deedee_driver.motors_driver_lib import *

class TestMotorsDriver(unittest.TestCase):
    def setUp(self):
        robot_name = "test_bot"
        config = {"max_wheel_speed": 0.55}

        serial_socket = Mock()
        self.md = MotorsDriver(robot_name, config, serial_socket)

    def test_speed_to_PWM_correct_range(self):
        '''
        '''
        max_wheel_speed = self.md.max_wheel_speed

        for i in range(-10,11):
            ratio = i / 10.
            expected_uniform_speed = ratio*100.
            uniform_speed = self.md.map_speed(ratio*max_wheel_speed)
            self.assertEqual(uniform_speed,
                             expected_uniform_speed,
                             "%s != %s" % (uniform_speed,
                                           expected_uniform_speed))

    def test_speed_to_PWM_over_range(self):
        '''
        '''
        max_wheel_speed = self.md.max_wheel_speed

        for i in range(10,101):
            ratio = i/10.
            expected_uniform_speed = 100.
            uniform_speed = self.md.map_speed(ratio*max_wheel_speed)
            self.assertEqual(uniform_speed,
                             expected_uniform_speed,
                             "%s != %s" % (uniform_speed,
                                           expected_uniform_speed))

        for i in range(-100,-9):
            ratio = i/10.
            expected_uniform_speed = -100.
            uniform_speed = self.md.map_speed(ratio*max_wheel_speed)
            self.assertEqual(uniform_speed,
                             expected_uniform_speed,
                             "%s != %s" % (uniform_speed,
                                           expected_uniform_speed))

    def test_build_packet(self):
        '''
        '''
        speed_left = 23
        speed_right = -100
        expected_packet = "$0,23,-100\n"
        packet = self.md.build_packet(speed_left, speed_right)
        self.assertEqual(packet,
                         expected_packet,
                         "%s != %s" % (packet,
                                       expected_packet))

if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, 'test_MotorsDriver', TestMotorsDriver)

