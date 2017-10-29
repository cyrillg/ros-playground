#! /usr/bin/env python

PKG='robot_control'

import sys
import unittest
from robot_control.movement_model_lib import *

class TestDiffDrive(unittest.TestCase):
  ''' Unit tests for the DiffDrive class
  '''

  def setUp(self):
    robot_name = "test_bot"
    config = {"L": .6,
              "r": .15}
    self.dd = DiffDrive(robot_name, config)

  def test_transform(self):
    ''' Test the output of transform function

        Cases:
        - Zero input
        - Spot turn
        - Straight forward
    '''
    inputs = [(0.,0.), (0., 12412.421), (-23431.142, 0.)]
    res = [self.dd.transform(i[0],i[1]) for i in inputs]

    self.assertEqual(res[0],
                     (0.,0.),
                     "Expected (0.,0.), got (u0,u1)=%s" % str(res[0]))
    self.assertEqual(res[1][0],
                     -res[1][1],
                     "Expected u0=-u2, got (u0,u1)=%s" % str(res[1]))
    self.assertEqual(res[2][0],
                     res[2][1],
                     "Expected u0=u2, got (u0,u1)=%s" % str(res[2]))


if __name__ == '__main__':
  import rosunit

  rosunit.unitrun(PKG, 'test_DiffDrive', TestDiffDrive)

