#!/usr/bin/env python

class DiffDrive:
  def __init__(self, L, r):
    self.L = L
    self.r = r

  def transform(self, v, w):
    ''' Transform linear and angular speed into wheel angular speeds
    '''
    u0 = (2*v + self.L*w) / (2*self.r)
    u1 = (2*v - self.L*w) / (2*self.r)

    return (u0, u1)

