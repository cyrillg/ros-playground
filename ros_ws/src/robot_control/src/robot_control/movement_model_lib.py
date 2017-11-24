#!/usr/bin/env python

class DiffDrive:
    def __init__(self, robot_name, config):
        self.L = config["L"]
        self.r = config["r"]
        print("\n\n\nL, r: {}, {}\n\n\n".format(self.L,self.r))

    def transform(self, v, w):
        ''' Transform linear and angular speed into wheel angular speeds
        '''
        u0 = (2*v + self.L*w) / (2*self.r)
        u1 = (2*v - self.L*w) / (2*self.r)

        return (u0, u1)

