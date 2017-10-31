#! /usr/bin/env python

PKG='robot_control'

import sys
import unittest

from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from robot_control.controller_lib import *

class TestPID(unittest.TestCase):
  ''' Unit tests for the PID class
  '''

  def test_pid_default_correct_output(self):
    ''' Test the output of default PID

        Expect the output to be equal to the input error, since
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

    self.p = PoseStamped()
    self.p.pose = pose2d_to_pose(Pose2D(1., 0., 3.*pi/4.))

  def test_LOS_aligned_target_front(self):
    ''' Test the output of the LOS algo

        Expect the error output to be equal to 0, since
        target is aligned.
    '''
    wp = PoseStamped()
    wp.pose = pose2d_to_pose(Pose2D(0., 1., 0.))

    err = self.los.compute_error(self.p, wp)
    self.assertEqual(err, 0, "Expected 0, got: %f" % err)

  def test_LOS_aligned_target_behind(self):
    ''' Test the output of the LOS algo

        It expects the error output to be equal to pi, since
        the target is exactly behind.
    '''
    wp = PoseStamped()
    wp.pose = pose2d_to_pose(Pose2D(2., -1., 0.))

    err = self.los.compute_error(self.p, wp)
    self.assertEqual(abs(err), pi, "Expected pi, got: %f" % abs(err))

  def test_LOS_same_point(self):
    ''' Test the output of the LOS algo

        It expects the error output to be equal to the 0, as special case
        if robot and target are at the same location.
    '''
    wp = PoseStamped()
    wp.pose = pose2d_to_pose(Pose2D(self.p.pose.position.x,
                                    self.p.pose.position.y,
                                    0.))

    err = self.los.compute_error(self.p, wp)
    self.assertEqual(err, 0, "Expected 0, got: %f" % err)

class TestController(unittest.TestCase):
  ''' Unit tests for the Controller class
  '''

  def setUp(self):
    robot_name = "test_bot"
    config = {"robot_speed": 5.,
              "PID_gain": 2.}
    self.c = Controller(robot_name, config)

    self.empty_path = Path()
    self.path1 = Path()
    self.path1.header.seq = 1
    self.path1.poses = [PoseStamped(pose=pose2d_to_pose(Pose2D(0.,1.,0.)),
                                    header=Header(seq=1)),
                        PoseStamped(pose=pose2d_to_pose(Pose2D(1.,0.,0.)),
                                    header=Header(seq=2)),
                        PoseStamped(pose=pose2d_to_pose(Pose2D(2.,3.,0.)),
                                    header=Header(seq=3))]
    self.path2 = Path()
    self.path2.header.seq = 2
    self.path2.poses = [PoseStamped(pose=pose2d_to_pose(Pose2D(1.,1.,0.)),
                                    header=Header(seq=1)),
                        PoseStamped(pose=pose2d_to_pose(Pose2D(1.,0.,0.)),
                                    header=Header(seq=2)),
                        PoseStamped(pose=pose2d_to_pose(Pose2D(2.,3.,0.)),
                                    header=Header(seq=3))]

  def test_controller_activate_success(self):
    ''' Test the activation of the controller on valid new path
    '''
    self.assertFalse(self.c.active,
                     "Expected inactive Controller at initialization")
    self.c.start(self.path1)
    self.assertTrue(self.c.active,
                    "Expected active Controller after start")
    self.assertEqual(self.c.current_wp.header.seq,
                     1,
                     "Expected current waypoint to be first waypoint")
    self.assertEqual(self.c.wp_idx,
                     1,
                     "Expected current idx to be 1")

  def test_controller_discard_empty_path(self):
    ''' Test the non-activation of the controller on a new empty path
    '''
    self.c.start(self.empty_path)
    self.assertFalse(self.c.active,
                     "Expected inactive Controller after start on empty path")

  def test_controller_discard_path_when_active(self):
    ''' Test that controller ignores a new path if it is already on one
    '''
    self.c.start(self.path1)
    self.c.start(self.path2)
    self.assertTrue(self.c.active,
                    "Expected active Controller after start")
    self.assertEqual(self.c.path, self.path1.poses,
                     "Expected first path to be active. Got second one.")

  def test_controller_keep_waypoint_if_too_far(self):
    ''' Test that controller does not switch to next point if too far from it
    '''
    self.c.start(self.path1)
    p = PoseStamped(pose=pose2d_to_pose(Pose2D(0., 0.79, 0.)))

    original_wp_idx = self.c.current_wp.header.seq
    self.c.supervise(0., p)
    self.assertEqual(self.c.current_wp.header.seq,
                     original_wp_idx,
                     "Expected waypoint to stay the same")

  def test_controller_next_waypoint_if_close_enough(self):
    ''' Test that controller switches to next point if close enough from it
    '''
    self.c.start(self.path1)
    p = PoseStamped(pose=pose2d_to_pose(Pose2D(0., 0.81, 0.)))

    self.c.supervise(0., p)
    self.assertEqual(self.c.current_wp.header.seq,
                     2,
                     "Expected waypoint to change to next one")

  def test_controller_reset_on_end_path(self):
    ''' Test correct reset of controller upon path completion
    '''
    self.c.start(self.path1)

    poses = self.path1.poses

    self.c.supervise(0., poses[0])
    self.c.supervise(0., poses[1])
    self.c.supervise(0., poses[2])
    self.assertIsNone(self.c.current_wp,
                     "Expected current_wp=None, got: %s" % self.c.current_wp)
    self.assertIsNone(self.c.wp_idx,
                     "Expected wp_idx=None, got: %s" % self.c.wp_idx)
    self.assertFalse(self.c.active,
                     "Expected inactive Controller after final target reached")

  def test_controller_standby_if_active_but_no_pose(self):
    ''' Test that controller generates zero speeds if active without pose
    '''
    self.c.start(self.path1)
    cmd = self.c.generate_cmd(0., None)
    self.assertTupleEqual(cmd,
                     (0., 0.),
                     "Expected speed=(0,0), got: %s" % str(cmd))

  def test_controller_standby_if_not_active(self):
    ''' Test that controller generates zero speeds if not active
    '''
    p = PoseStamped()
    p.pose = pose2d_to_pose(Pose2D(0., 0.81, 0.))
    cmd = self.c.generate_cmd(0., p)
    self.assertTupleEqual(cmd,
                     (0., 0.),
                     "Expected speed=(0,0), got: %s" % str(cmd))

  def test_controller_standby_if_target_reached(self):
    ''' Test that controller generates zero speeds upon path completion
    '''
    self.c.start(self.path1)

    poses = self.path1.poses

    self.c.supervise(0., poses[0])
    self.c.supervise(0., poses[1])
    self.c.supervise(0., poses[2])

    cmd = self.c.generate_cmd(0., poses[2])
    self.assertTupleEqual(cmd,
                     (0., 0.),
                     "Expected speed=(0,0), got: %s" % str(cmd))

  def test_controller_moving_if_active(self):
    ''' Test that controller generates non-zero speeds when active with pose
    '''
    self.c.start(self.path1)

    poses = self.path1.poses

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

