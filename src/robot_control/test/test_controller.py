#!/usr/bin/env python

PKG='robot_control'
#import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import time
import unittest
from robot_control.controller_lib import *

# Unit test for the PID class
class TestPID(unittest.TestCase):

  def test_pid_default_correct_output(self):
    ''' Test the output of default PID

        It expects the output to be equal to the input error, since
        the default PID has a gain of 1
    '''
    pid = PID()
    err = [3.5, -12.34, 0, 1829304839210938, -128921939071230]
    res = [pid.P(e) for e in err]
    for i in range(len(err)):
      self.assertEquals(err[i], res[i], "Expected %f, got %f" % (err[i],
                                                                 res[i]))

  def test_pid_correct_output(self):
    ''' Test the output of a PID with custom gain
    '''
    K = 12
    pid = PID(K)
    err = [3.5, -12.34, 0, 1829304839210938, -128921939071230]
    for i in range(len(err)):
      res = [pid.P(e) for e in err]
      self.assertEquals(K*err[i], res[i], "Expected %f, got %f" % (K*err[i],
                                                                   res[i]))

# Unit test for the LOS class
class TestLOS(unittest.TestCase):

  def setUp(self):
    self.los = LOS()
    self.p = Pose2D(1.0, 0.0, 3*pi/4)

  def test_LOS_aligned_target_front(self):
    ''' Test the output of the LOS algo

        It expects the error output to be equal to the 0, since
        the default PID has a gain of 1
    '''
    wp = [0.0, 1.0]

    err = self.los.compute_error(self.p, wp)
    self.assertEquals(err, 0, "Expected 0, got: %f" % err)

  def test_LOS_aligned_target_behind(self):
    ''' Test the output of the LOS algo

        It expects the error output to be equal to the 0, since
        the default PID has a gain of 1
    '''
    wp = [2.0, -1.0]

    err = self.los.compute_error(self.p, wp)
    self.assertEquals(abs(err), pi, "Expected pi, got: %f" % abs(err))

  def test_LOS_same_point(self):
    ''' Test the output of the LOS algo

        It expects the error output to be equal to the 0, since
        the default PID has a gain of 1
    '''
    wp = [self.p.x, self.p.y]

    err = self.los.compute_error(self.p, wp)
    self.assertEquals(err, 0, "Expected 0, got: %f" % err)

# Unit test for the Controller class
class TestController(unittest.TestCase):

  def setUp(self):
    self.los = LOS()
    self.p = Pose2D(1.0, 0.0, 3*pi/4)


if __name__ == '__main__':
  import rosunit

  rosunit.unitrun(PKG, 'test_PID', TestPID)
  rosunit.unitrun(PKG, 'test_LOS', TestLOS)
