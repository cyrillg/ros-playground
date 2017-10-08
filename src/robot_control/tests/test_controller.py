#! /usr/bin/env python

PKG='robot_control'

import sys
import unittest
from robot_control.controller_lib import *

class TestPID(unittest.TestCase):
  ''' Unit tests for the PID class
  '''

  def test_pid_default_correct_output(self):
    ''' Test the output of default PID

        It expects the output to be equal to the input error, since
        the default PID has a gain of 1
    '''
    pid = PID()
    err = [3.5, -12.34, 0, 1829304839210938, -128921939071230]
    res = [pid.P(e) for e in err]
    for i in range(len(err)):
      self.assertEqual(err[i], res[i], "Expected %f, got %f" % (err[i],
                                                                 res[i]))

  def test_pid_correct_output(self):
    ''' Test the output of a PID with custom gain
    '''
    K = 12
    pid = PID(K)
    err = [3.5, -12.34, 0, 1829304839210938, -128921939071230]
    for i in range(len(err)):
      res = [pid.P(e) for e in err]
      self.assertEqual(K*err[i], res[i], "Expected %f, got %f" % (K*err[i],
                                                                   res[i]))

class TestLOS(unittest.TestCase):
  ''' Unit tests for the LOS class
  '''

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
    self.assertEqual(err, 0, "Expected 0, got: %f" % err)

  def test_LOS_aligned_target_behind(self):
    ''' Test the output of the LOS algo

        It expects the error output to be equal to the 0, since
        the default PID has a gain of 1
    '''
    wp = [2.0, -1.0]

    err = self.los.compute_error(self.p, wp)
    self.assertEqual(abs(err), pi, "Expected pi, got: %f" % abs(err))

  def test_LOS_same_point(self):
    ''' Test the output of the LOS algo

        It expects the error output to be equal to the 0, since
        the default PID has a gain of 1
    '''
    wp = [self.p.x, self.p.y]

    err = self.los.compute_error(self.p, wp)
    self.assertEqual(err, 0, "Expected 0, got: %f" % err)

class TestController(unittest.TestCase):
  ''' Unit tests for the Controller class
  '''

  def setUp(self):
    self.c = Controller()

    self.empty_path = []
    self.path1 = [[0.,1.],
                  [1.,0.],
                  [2.,3.]]
    self.path2 = [[1.,1.],
                  [1.,0.],
                  [2.,3.]]

  def test_controller_activate_success(self):
    self.assertFalse(self.c.active,
                     "Expected inactive Controller at initialization")
    self.c.start(self.path1)
    self.assertTrue(self.c.active,
                    "Expected active Controller after start")
    self.assertEqual(self.c.current_wp,
                     self.path1[0],
                     "Expected current waypoint to be first waypoint")
    self.assertEqual(self.c.wp_idx,
                     1,
                     "Expected current idx to be 1")

  def test_controller_discard_empty_path(self):
    self.c.start(self.empty_path)
    self.assertFalse(self.c.active,
                     "Expected inactive Controller after start on empty path")

  def test_controller_discard_path_when_active(self):
    self.c.start(self.path1)
    self.c.start(self.path2)
    self.assertTrue(self.c.active,
                    "Expected active Controller after start")
    self.assertEqual(self.c.path, self.path1,
                     "Expected first path to be active. Got second one.")

  def test_controller_keep_waypoint_if_too_far(self):
    self.c.start(self.path1)
    p = Pose2D(0., 0.79, 0.)

    original_wp = self.c.current_wp
    self.c.supervise(0., p)
    self.assertEqual(self.c.current_wp,
                     original_wp,
                     "Expected waypoint to stay the same")

  def test_controller_next_waypoint_if_close_enough(self):
    self.c.start(self.path1)
    p = Pose2D(0., 0.81, 0.)

    expected_wp = self.path1[self.c.wp_idx]
    self.c.supervise(0., p)
    self.assertEqual(self.c.current_wp,
                     expected_wp,
                     "Expected waypoint to change to next one")

  def test_controller_reset_on_end_path(self):
    self.c.start(self.path1)

    poses = [Pose2D(p[0], p[1], 0.) for p in self.path1]

    self.c.supervise(0., poses[0])
    self.c.supervise(0., poses[1])
    self.c.supervise(0., poses[2])
    self.assertEqual(self.c.current_wp,
                     None,
                     "Expected current_wp=None, got: %s" % self.c.current_wp)
    self.assertEqual(self.c.wp_idx,
                     1,
                     "Expected wp_idx=1, got: %s" % self.c.wp_idx)
    self.assertFalse(self.c.active,
                     "Expected inactive Controller after final target reached")

  def test_controller_standby_if_no_pose(self):
    cmd = self.c.generate_cmd(0., None)
    self.assertEqual(cmd,
                     (0., 0.),
                     "Expected speed=(0,0), got: %s" % str(cmd))

  def test_controller_standby_if_not_active(self):
    p = Pose2D(0., 0.81, 0.)
    cmd = self.c.generate_cmd(0., p)
    self.assertEqual(cmd,
                     (0., 0.),
                     "Expected speed=(0,0), got: %s" % str(cmd))

  def test_controller_standby_if_target_reached(self):
    self.c.start(self.path1)

    poses = [Pose2D(p[0], p[1], 0.) for p in self.path1]

    self.c.supervise(0., poses[0])
    self.c.supervise(0., poses[1])
    self.c.supervise(0., poses[2])

    cmd = self.c.generate_cmd(0., poses[2])
    self.assertEqual(cmd,
                     (0., 0.),
                     "Expected speed=(0,0), got: %s" % str(cmd))

  def test_controller_moving_if_active(self):
    self.c.start(self.path1)

    poses = [Pose2D(p[0], p[1], 0.) for p in self.path1]

    self.c.supervise(0., poses[0])

    cmd = self.c.generate_cmd(0., poses[0])
    self.assertNotEqual(cmd,
                        (0., 0.),
                        "Expected speed!=(0,0), got: (0,0)")


if __name__ == '__main__':
  import rosunit

  rosunit.unitrun(PKG, 'test_PID', TestPID)
  rosunit.unitrun(PKG, 'test_LOS', TestLOS)
  rosunit.unitrun(PKG, 'test_Controller', TestController)
