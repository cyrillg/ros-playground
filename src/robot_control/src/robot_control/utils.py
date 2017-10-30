'''
Definition of helper function

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from numpy import sqrt, arctan2, pi
from geometry_msgs.msg import Pose, Pose2D
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
  ret_pose.theta = rpy_from_q(pose.orientation)[2]

  return ret_pose

def pose2d_to_pose(pose):
  ret_pose = Pose()

  ret_pose.position.x = pose.x
  ret_pose.position.y = pose.y
  ret_pose.orientation = q_from_rpy(0., 0., pose.theta)

  return ret_pose

