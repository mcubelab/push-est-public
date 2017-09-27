#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# Use knn to predict motion of the object given push location and velocity

import numpy as np
import matplotlib.pyplot as plt

from sklearn.datasets import fetch_olivetti_faces
from sklearn.utils.validation import check_random_state

from sklearn.ensemble import ExtraTreesRegressor
from sklearn.neighbors import KNeighborsRegressor
from sklearn.linear_model import LinearRegression
from sklearn.linear_model import RidgeCV

import sys, json
from ik.helper import *

def knn(inputfile, k):
    
    
    # prepare data
    
    #inputfile= "%s/data_training_with_vel.json" % argv[1]
    
    #inputfile= argv[1]
    # the data is a 2-d array with these labels for columns
    labels = ['tip_x', 'tip_y', 'tip_vx', 'tip_vy', 'forcex', 'forcey', 'object_pose_vx', 'object_pose_vy', 'object_pose_vtheta']
    
    
    # tip_x, tip_y, tip_dx, tip_dy, forcex, forcey, object_pose_dx, object_pose_dy, object_pose_dtheta, 
    # tip_svx, tip_svy, tip_evx, tip_evy, 
    # object_pose_svx, object_pose_svy, object_pose_svtheta, # start speed
    # object_pose_evx, object_pose_evy, object_pose_evtheta  # end speed
    
    with open(inputfile) as data_file:    
        data = json.load(data_file)
    
    data = np.random.permutation(data).tolist()
    
    #k = int(argv[2])
    
    # cross validation
    n_cross = 5
    n_data = len(data)
    n_perseg = n_data/n_cross
    error_xy = 0
    error_angle = 0
    error_vxy = 0
    error_vangle = 0
    for i in range(n_cross):
        test_seg_begin = i * n_perseg
        test_seg_end = ((i+1) * n_perseg) if (i < n_cross - 1) else (n_data)
        n_test = test_seg_end - test_seg_begin
        
        X_train = np.array(data[0:test_seg_begin] + data[test_seg_end:n_data])[:,0:4]
        #X_train = np.hstack((X_train, np.array(data[0:test_seg_begin] + data[test_seg_end:n_data])[:,13:16])).tolist()  # comment this out for static model
        
        y_train = np.array(data[0:test_seg_begin] + data[test_seg_end:n_data])[:,6:9]
        y_train = np.hstack((y_train, np.array(data[0:test_seg_begin] + data[test_seg_end:n_data])[:,16:19])).tolist()
        
        
        X_test = np.array(data[test_seg_begin:test_seg_end])[:,0:4]
        #X_test = np.hstack((X_test, np.array(data[test_seg_begin:test_seg_end])[:,13:16])).tolist() # comment this out for static model
        y_test = np.array(data[test_seg_begin:test_seg_end])[:,6:9]
        y_test = np.hstack((y_test, np.array(data[test_seg_begin:test_seg_end])[:,16:19])).tolist()
        
        estimator = KNeighborsRegressor(n_neighbors=k)
        estimator.fit(X_train, y_train)
        y_test_predict = estimator.predict(X_test)
        
        error_xy += norm((np.array(y_test_predict) - np.array(y_test))[:,0:2].flatten(1)) / n_test
        error_angle += norm((np.array(y_test_predict) - np.array(y_test))[:,2].flatten(1)) / n_test
        
        error_vxy += norm((np.array(y_test_predict) - np.array(y_test))[:,3:5].flatten(1)) / n_test
        error_vangle += norm((np.array(y_test_predict) - np.array(y_test))[:,5].flatten(1)) / n_test
            
    error_xy /= n_cross
    error_angle /= n_cross

    std_xy = (norm(np.array(data)[:,6:8].flatten(1))**2 / n_data)**0.5
    std_angle = (norm(np.array(data)[:,8].flatten(1))**2 / n_data)**0.5
    
    std_vxy = (norm(np.array(data)[:,16:18].flatten(1))**2 / n_data)**0.5
    std_vangle = (norm(np.array(data)[:,18].flatten(1))**2 / n_data)**0.5
    
    #print(data[:4])
    #plt.plot(np.array(data)[:,6].tolist())
    #plt.show()

    #print 'error_xy %.2f (micrometer)' % (error_xy*(10**6)), 'error_angle %.2f (10^-5 rad)' % (error_angle*(10**5))
    #print 'std_xy', std_xy, 'std_angle', std_angle
    #print 'error_xy_percent %.2f%% ' % (error_xy/std_xy*100), 'error_angle_percent %.2f %%' % (error_angle/std_angle*100)
    
    # print 'k', k
    # print 'error_vxy %.2f (micrometer)' % (error_vxy*(10**6)), 'error_vangle %.2f (10^-5 rad)' % (error_vangle*(10**5))
    # print 'std_vxy', std_vxy, 'std_vangle', std_vangle
    # print 'error_vxy_percent %.2f%% ' % (error_vxy/std_vxy*100), 'error_vangle_percent %.2f %%' % (error_vangle/std_vangle*100)

    print '%d & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f\\\\ \hline' % (k, error_xy*(10**6), error_angle*(10**5), error_xy/std_xy*100, error_angle/std_angle*100,
           error_vxy*(10**6), error_vangle*(10**5), error_vxy/std_vxy*100, error_vangle/std_vangle*100)
    
def main(argv):
    for k in range(1,10,2):
        knn(argv[1],k)

if __name__=='__main__':
    main(sys.argv)



