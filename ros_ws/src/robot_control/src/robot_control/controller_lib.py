#!/usr/bin/env python

from robot_control.utils import *

import rospy
from tf.transformations import euler_from_quaternion as rpy_from_q

class Controller:
    ''' Closed loop "waypoint path follower" controller definition

        Inputs:
          - reference: sequence of (x, y) waypoints. Must be specified as a
                       list of tuples.
    '''
    def __init__(self, robot_name, config):
        # Configs
        self.v = config["robot_speed"] #.5
        PID_gain = config["PID_gain"] #2.

        self.hi_lvl_ctrl = LOS()
        self.lo_lvl_ctrl = PID(PID_gain)

        self.path = None
        self.active = False
        self.wp_idx = None
        self.current_wp = None

        rospy.loginfo("\nController initialized for {}, awaiting path".format(robot_name))

    def start(self, path):
        header = path.header
        path = path.poses

        if path:
            if not self.active:
                self.path = path
                self.wp_idx = 1
                self.current_wp = self.path[0]

                rospy.loginfo("Controller activated")
                rospy.loginfo("  Path composed of {} waypoints".format(len(path)))
                rospy.loginfo("\nInitial target: {}. {}".format(self.wp_idx-1,
                                                        self.current_wp))
                self.active = True
            else:
                rospy.loginfo("Path already in progress. Discarding new path.")
        else:
            rospy.loginfo("Discarding empty path.")

    def supervise(self, t, p):
        ''' Supervisor handling waypoint switching
        '''
        current_wp = self.current_wp

        dist = sqrt(pow(p.pose.position.x - current_wp.pose.position.x, 2)
                    +pow(p.pose.position.y - current_wp.pose.position.y, 2))
        if dist<0.2:
            if self.wp_idx<len(self.path):
                self.wp_idx += 1
                self.current_wp = self.path[self.wp_idx-1]
                rospy.loginfo("New target: {}. {}".format(self.wp_idx-1,
                                                          current_wp))
            else:
                self.active = False
                self.current_wp = None
                self.wp_idx = None
                rospy.loginfo("\nFinal target reached\n")

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
        th_ref = arctan2(wp.pose.position.y - p.pose.position.y,
                         wp.pose.position.x - p.pose.position.x)
        if th_ref!=0:
            q = p.pose.orientation
            th = rpy_from_q((q.x, q.y, q.z, q.w))[2]
            th_err = normalize(th_ref-th)
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

