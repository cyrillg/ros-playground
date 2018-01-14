#!/usr/bin/env python

from numpy import sqrt, arctan2, pi
from geometry_msgs.msg import Pose, Pose2D, Quaternion
from tf.transformations import euler_from_quaternion as rpy_from_q
from tf.transformations import quaternion_from_euler as q_from_rpy

def normalize(angle):
    ''' Normalize an angle in radians between -pi and pi

        Inputs:
          - angle: angle to normalize
    '''
    angle = angle%(2*pi)
    if angle>pi:
        angle -= 2*pi
    return angle

def pose_to_pose2d(pose):
    ret_pose = Pose2D()

    ret_pose.x = pose.position.x
    ret_pose.y = pose.position.y
    q = pose.orientation
    ret_pose.theta = rpy_from_q(q[0], q[1], q[2], q[3])[2]

    return ret_pose

def pose2d_to_pose(pose):
    ret_pose = Pose()

    ret_pose.position.x = pose.x
    ret_pose.position.y = pose.y
    q = q_from_rpy(0., 0., pose.theta)
    ret_pose.orientation = Quaternion(x=q[0],
                                      y=q[1],
                                      z=q[2],
                                      w=q[3])

    return ret_pose

