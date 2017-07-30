#!/usr/bin/env python

import rospy
from robot_control.utils import *
from copy import deepcopy
from rospy import Subscriber, Publisher
from tf.transformations import euler_from_quaternion as rpy_from_q
from geometry_msgs.msg import Pose, Pose2D, Twist
from gazebo_msgs.msg import ModelStates

class Controller:
  ''' Closed loop controller definition

      Inputs:
        - reference: sequence of (x, y) waypoints. Must be specified as a
                     list of tuples.
  '''
  def __init__(self,
               reference=[(2.0, 0.0),
                          (3.0, -1.0),
                          (0.0, 0.0)]):
    rospy.init_node("controller")
    rospy.sleep(0.5)

    self.type = "closed-loop"
    self.path = reference
    self.K = 2.
    self.v = .5
    self.is_end = False
    self.sim_timeout = 60.0
    self.p = None

    self.wp_idx = 1
    self.current_wp = self.path[0]

    self.pose_sub = Subscriber("/gazebo/model_states",
                               ModelStates,
                               self.on_model_states)
    self.cmd_vel_pub = Publisher("/cmd_vel",
                                 Twist,
                                 queue_size=10)

    print("\nController launched")
    print("  Path composed of {} waypoints".format(len(self.path)))
    print("\nInitial target: {}. {}".format(self.wp_idx-1,
                                            self.current_wp))

    self.run()

  def on_model_states(self, state):
    quaternion = (state.pose[1].orientation.x,
                  state.pose[1].orientation.y,
                  state.pose[1].orientation.z,
                  state.pose[1].orientation.w)
    rpy = rpy_from_q(quaternion)
    self.p = Pose2D(x=state.pose[1].position.x,
                    y=state.pose[1].position.y,
                    theta=rpy[2])

  def run(self):
    rate = rospy.Rate(10)
    t_start = rospy.Time.now().to_sec()
    while not self.is_end and not rospy.is_shutdown():
      t = rospy.Time.now().to_sec() - t_start
      self.generate_cmd(t)
      rate.sleep()

  def generate_cmd(self, t):
    ''' Main function to generate the current wheel angular speed inputs
    '''
    if self.p:
      p = deepcopy(self.p)
      self.current_wp = self.supervise(t, p)

      if not self.is_end:
        th_err = self.LOS(p, self.current_wp)

        v = self.v
        w = self.P(self.K, th_err)

      else:
        v = 0
        w = 0

      msg = Twist()
      msg.linear.x = v
      msg.angular.z = w
      self.cmd_vel_pub.publish(msg)

  def supervise(self, t, p):
    ''' Supervisor handling waypoint switching and simulation end
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
      elif not self.is_end:
        self.is_end = True
        current_wp = None
        if t>=self.sim_timeout:
          print "Simulation timeout reached"
        else:
          print "\nFinal target reached\n"

    return current_wp

  def LOS(self, p, wp):
    ''' Guidance law to generate the heading reference
    '''
    th_ref = arctan2(wp[1]-p.y,wp[0]-p.x)
    th_err = normalize(th_ref-p.theta)

    return th_err

  def P(self, K, err):
    ''' P controller to generate the angular speed command
    '''
    return K*err

if __name__=="__main__":
  Controller()

