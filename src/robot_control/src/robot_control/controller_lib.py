#!/usr/bin/env python

from robot_control.utils import *
from copy import deepcopy
from geometry_msgs.msg import Pose2D, Twist

class Controller:
  ''' Closed loop "waypoint path follower" controller definition

      Inputs:
        - reference: sequence of (x, y) waypoints. Must be specified as a
                     list of tuples.
  '''
  def __init__(self):
    self.path = None
    self.active = False

    self.wp_idx = 1
    self.current_wp = None

    self.hi_lvl_ctrl = LOS()
    self.lo_lvl_ctrl = PID(2.)

    self.v = .5

    print("\nController initialized, awaiting path")

  def start(self, path):
    if path:
      if not self.active:
        print("Controller activated")
        print("  Path composed of {} waypoints".format(len(path)))
        print("\nInitial target: {}. {}".format(self.wp_idx-1,
                                                self.current_wp))
        self.path = path
        self.wp_idx = 1
        self.current_wp = self.path[0]
        self.active = True
      else:
        print("Path already in progress. Discarding new path.")
    else:
      print("Discarding empty path.")

  def supervise(self, t, p):
    ''' Supervisor handling waypoint switching
    '''
    current_wp = self.current_wp

    dist = sqrt(pow(p.x-current_wp[0],2)
                +pow(p.y-current_wp[1],2))
    if dist<0.2:
      if self.wp_idx<len(self.path):
        self.wp_idx += 1
        self.current_wp = self.path[self.wp_idx-1]
        print("New target: {}. {}".format(self.wp_idx-1,
                                          current_wp))
      else:
        self.current_wp = None
        self.wp_idx = 1
        self.active = False
        print "\nFinal target reached\n"

  def generate_cmd(self, t, p):
    ''' Generate the current wheel angular speed inputs
    '''
    if p and self.active:
      self.supervise(t, p)

      if self.active:
        th_err = self.hi_lvl_ctrl.compute_error(p, self.current_wp)

        v = self.v
        w = self.lo_lvl_ctrl.P(th_err)
      else:
        v = 0
        w = 0

    else:
      v = 0
      w = 0

    return (v, w)


class LOS:
  ''' Guidance law to generate the heading reference
  '''
  def compute_error(self, p, wp):
    th_ref = arctan2(wp[1]-p.y,wp[0]-p.x)
    if th_ref!=0:
      th_err = normalize(th_ref-p.theta)
    else:
      th_err = 0

    return th_err


class PID:
  ''' P controller to generate the angular speed command
  '''
  def __init__(self, K=1):
    self.K = K

  def P(self, err):
    return self.K*err

