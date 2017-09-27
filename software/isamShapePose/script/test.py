#!/usr/bin/env python
from ik.helper import *

import config.shape_db

def wraptopi(data):
    return ( data + np.pi) % (2 * np.pi ) - np.pi 

def f(d):
    x = (d[0], d[1])
    vel = (d[2], d[3])
    motion = (d[6], d[7], d[8])
    
    # find which side
    if x[0] + x[1] >= 0 and x[0] - x[1] <= 0:
        sth = 0.0 
    elif x[0] + x[1] <= 0 and x[0] - x[1] <= 0:
        sth = np.pi/2
    elif x[0] + x[1] <= 0 and x[0] - x[1] >= 0:
        sth = np.pi ; 
    elif x[0] + x[1] >= 0 and x[0] - x[1] >= 0:
        sth = np.pi/2.0*3 ; 
    else:
        print 'bad'
    
    
    sth -= np.pi/2
    
    # convert it to side 0
    x_side0 = rotate_to_frame2d(x, [0,0,sth])
    vel_side0 = rotate_to_frame2d(vel, [0,0,sth])
    
    # find c
    c = (x[1] + 0.045) / 0.09  # todo: need to be moved out
    c = max( min(1, c), 0)  # cap in [0,1]
    
    # find beta from vel
    beta = wraptopi(np.arctan2(vel_side0[1], vel_side0[0]))  # careful about to find the right sign
    
    motion_side0 = rotate_to_frame2d(motion[0:2], [0,0,sth])
    motion_side0 = motion_side0 + [wraptopi(motion[2])]
    print sth, x_side0, vel_side0, c, beta, motion_side0


#f([-0.2, 0.1, 1,1, 0,0, 0.1, 0.3, 0.05])


sdb = config.shape_db.ShapeDB();
butter = sdb.shape_db["butter"]["shape"][0]
for x in butter:
    #print x 
    print "{%.8f,%.8f}," % (x[0],x[1])
print len(butter)

