'''
Definition of helper function

author: Cyrill Guillemot
email: cyrill.guillemot@gmail.com
website: http://serial-robotics.org
license: GNU GPL
'''

#!/usr/bin/env python

from numpy import sqrt, arctan2, pi

def normalize(angle):
    ''' Normalize an angle in radians between -pi and pi

        Inputs:
          - angle: angle to normalize
    '''
    angle = angle%(2*pi)
    if angle>pi:
        angle -= 2*pi
    return angle
