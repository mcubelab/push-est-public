from config.shape_db import ShapeDB
import numpy as np
from numpy import *

# check whether the probe will hit the object

# def polyapprox_check_inside(shape, point):
    # n = len(shape)
    # for i in xrange(0,n,10):
        # if cross((np.array(shape[(i+1)%n]) - np.array(shape[i])) , (np.array(point)-np.array(shape[i]))) < 0:
            # return False
            # 
    # return True
    
def polyapprox_check_inside(shape, point):
    ss = shape
    # find the closest point on the shape to pos_start_probe_object
    min_dist = 10000
    min_ind = 0
    for i in range(len(ss)):
        vec = np.array(ss[i]) - np.array(point)
        dist = dot(vec, vec)
        if dist < min_dist:
            min_dist = dist
            min_ind = i
            
    tangent = np.array(ss[(min_ind+1) % len(ss)])-np.array(ss[min_ind])
    normal = np.array([tangent[1], -tangent[0]]) # pointing out of shape
    d = np.dot(normal, np.array(point) - np.array(ss[min_ind]) )
    if d < 0:
        return True
    else:
        return False

def poly_check_inside(shape, point):
    n = len(shape)
    for i in xrange(n):
        if cross((np.array(shape[(i+1)%n]) - np.array(shape[i])) , (np.array(point)-np.array(shape[i]))) < 0:
            return False
            
    return True

def ellip_check_inside(shape, point):
    a = shape[0][0]
    b = shape[0][1]
    return (point[0]**2 / a**2  + point[1]**2 / b**2) < 1
    
def main(argv):
    resolution = 0.0002
    shape_db = ShapeDB()
    for shape_id, value in shape_db.shape_db.iteritems():
        #if not shape_id == 'butter': continue
        shape_type = value['shape_type']
        if shape_type in ['poly', 'ellip', 'polyapprox']:
            # find the boundary limits
            shape = value["shape"]
            if shape_type == "polyapprox": shape = shape[0]
            if shape_type in ['poly']:
                xy = zip(*shape)
                bounds = [np.min(xy[0]), np.max(xy[0]), np.min(xy[1]), np.max(xy[1])]
                check_inside = poly_check_inside
            elif shape_type in ['polyapprox']:
                xy = zip(*shape)
                bounds = [np.min(xy[0]), np.max(xy[0]), np.min(xy[1]), np.max(xy[1])]
                check_inside = polyapprox_check_inside
            elif shape_type == 'ellip':
                bounds = [-shape[0][0], shape[0][0], -shape[0][1], shape[0][1]]
                check_inside = ellip_check_inside
                
            # discretize in the bounds every 0.0001 m
            xs = arange(bounds[0], bounds[1], resolution)
            ys = arange(bounds[2], bounds[3], resolution)
            
            sumxy = np.array([0,0])
            n_point = 0
            
            # enumerate all the points
            # find rc by sum up (x,y) divided by npoints
            
            if shape_type == "polyapprox":
                sum_rsq = 0
                for x in xs:
                    for y in ys:
                        if check_inside(shape, (x,y)):
                            n_point += 1
                            sum_rsq += dot(np.array([x,y]), np.array([x,y]))
                moment = sum_rsq / n_point * value['mass']
                cxy = array([0,0])
                
            else:
                xys = []
                for x in xs:
                    for y in ys:
                        if check_inside(shape, (x,y)):
                            sumxy = sumxy + np.array([x,y])
                            n_point += 1
                            xys.append((x,y))
                            
                cxy = sumxy / n_point
                
                if not('tri' in shape_id):  # then symetric
                    cxy = array([0,0])
                    
                
                # sum up (r-rc)^2 in boundary
                # times m/npoints in the boundary
                sum_rsq = 0
                for (x,y) in xys:
                    sum_rsq += dot(np.array([x,y])-cxy, np.array([x,y])-cxy)
                
                moment = sum_rsq / n_point * value['mass']
            
            print shape_id, '\n', 'centroid', cxy, 'moment', moment * 1000, 'n_point', n_point

import sys
if __name__=='__main__':
    main(sys.argv)
