#!/usr/bin/env python

import rospy
from robot_control.utils import *
from copy import deepcopy
from rospy import Subscriber, Publisher
from geometry_msgs.msg import Pose2D, Twist

class ControllerNode:
  ''' Controller node
  '''
  def __init__(self,
               reference=[(2.0, 0.0),
                          (3.0, -1.0),
                          (0.0, 0.0)]):
    rospy.init_node("controller")
    rospy.sleep(0.5)

    self.controller = Controller()

    self.is_end = False
    self.sim_timeout = 60.0
    self.p = None

    self.pose_sub = Subscriber("state",
                               Pose2D,
                               self.on_pose)
    self.cmd_vel_pub = Publisher("twist_cmd",
                                 Twist,
                                 queue_size=10)

    self.run()

  def on_pose(self, pose):
    ''' Store the incoming new pose
    '''
    self.p = pose

  def run(self):
    ''' Main loop
    '''
    rate = rospy.Rate(10)
    t_start = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
      if self.controller.active:
        t = rospy.Time.now().to_sec() - t_start
        (v, w) = self.controller.generate_cmd(t, p)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

        rate.sleep()


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
    self.p = None

    print("\nController initialized, awaiting path")

  def start(path):
    if not self.active:
      print("Controller activated")
      print("  Path composed of {} waypoints".format(len(self.path)))
      print("\nInitial target: {}. {}".format(self.wp_idx-1,
                                              self.current_wp))
      self.path = path
      self.wp_idx = 1
      self.current_wp = self.path[0]
      self.active = True
    else:
      print("Path already in progress. Discarding new path.")

  def generate_cmd(self, t, p):
    ''' Generate the current wheel angular speed inputs
    '''
    if p:
      self.current_wp = self.supervise(t, p)

      if self.current_wp:
        th_err = self.hi_lvl_ctrl.compute_error(p, self.current_wp)

        v = self.v
        w = self.lo_lvl_ctrl.PID(self.K, th_err)
      else:
        v = 0
        w = 0
        self.active = False

      return (v, w)

  def supervise(self, t, p):
  ''' Supervisor handling waypoint switching
  '''
    current_wp = self.current_wp

    dist = sqrt(pow(p.x-current_wp[0],2)
                +pow(p.y-current_wp[1],2))
    if dist<0.2:
      if self.wp_idx<len(self.path) and t<self.sim_timeout:
        self.wp_idx += 1
        current_wp = self.path[self.wp_idx-1]
        print("New target: {}. {}".format(self.wp_idx-1,
                                          current_wp))
      else:
        current_wp = None
        print "\nFinal target reached\n"

    return current_wp


class LOS:
  ''' Guidance law to generate the heading reference
  '''
  def compute_error(self, p, wp):
    th_ref = arctan2(wp[1]-p.y,wp[0]-p.x)
    th_err = normalize(th_ref-p.theta)

    return th_err


class PID:
  ''' P controller to generate the angular speed command
  '''
  def __init__(self, K=1):
    self.K = K

  def P(self, err):
    return self.K*err

