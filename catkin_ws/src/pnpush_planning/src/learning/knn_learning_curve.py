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

def main(argv):
    
    # prepare data
    
    inputfile= argv[1]
    # the data is a 2-d array with these labels for columns
    labels = ['tip_x', 'tip_y', 'tip_vx', 'tip_vy', 'forcex', 'forcey', 'object_pose_vx', 'object_pose_vy', 'object_pose_vtheta']
    
    with open(inputfile) as data_file:    
        origdata = json.load(data_file)
    
    origdata = np.random.permutation(origdata).tolist()
    
    k = int(argv[2])
    
    train_xy_errs = []
    train_angle_errs = []
    test_xy_errs = []
    test_angle_errs = []
    
    n_origdata = len(origdata)
    std_xy = (norm(np.array(origdata)[:,6:8].flatten(1))**2 / n_origdata)**0.5
    std_angle = (norm(np.array(origdata)[:,8].flatten(1))**2 / n_origdata)**0.5
    
    sub = 30
    #r = [len(origdata)-1]
    r = range(11, n_origdata, sub) + [len(origdata)-1]
    for j in r:
        print j
        data = origdata[0:(j+1)]
        # cross validation
        n_cross = 5
        n_data = len(data)
        n_perseg = n_data/n_cross
        test_error_xy = 0
        test_error_angle = 0
        train_error_xy = 0
        train_error_angle = 0
        
        for i in range(n_cross):
            test_seg_begin = i * n_perseg
            test_seg_end = ((i+1) * n_perseg) if (i < n_cross - 1) else (n_data )
            n_test = test_seg_end - test_seg_begin
            
            X_train = np.array(data[0:test_seg_begin] + data[test_seg_end:n_data])[:,0:4].tolist()
            y_train = np.array(data[0:test_seg_begin] + data[test_seg_end:n_data])[:,6:9].tolist()
            X_test = np.array(data[test_seg_begin:test_seg_end])[:,0:4].tolist()
            y_test = np.array(data[test_seg_begin:test_seg_end])[:,6:9].tolist()
            
            estimator = KNeighborsRegressor(n_neighbors=k)
            estimator.fit(X_train, y_train)
            y_test_predict = estimator.predict(X_test)
            y_train_predict = estimator.predict(X_train)
            
            test_error_xy += norm((np.array(y_test_predict) - np.array(y_test))[:,0:2].flatten(1)) / n_test
            test_error_angle += norm((np.array(y_test_predict) - np.array(y_test))[:,2].flatten(1)) / n_test
            
            train_error_xy += norm((np.array(y_train_predict) - np.array(y_train))[:,0:2].flatten(1)) / n_test
            train_error_angle += norm((np.array(y_train_predict) - np.array(y_train))[:,2].flatten(1)) / n_test
                
        test_error_xy /= n_cross
        test_error_angle /= n_cross

        train_error_xy /= n_cross
        train_error_angle /= n_cross

        train_xy_errs.append(train_error_xy/std_xy); train_angle_errs.append(train_error_angle/std_angle); 
        test_xy_errs.append(test_error_xy/std_xy); test_angle_errs.append(test_error_angle/std_angle); 

    print 'test_error_xy', test_xy_errs[-1]*std_xy, 'test_error_angle', test_angle_errs[-1]*std_angle
    #print 'var_xy', std_xy, 'std_angle', std_angle
    print 'error_xy_percent', test_xy_errs[-1], 'error_angle_percent', test_angle_errs[-1]

    f = plt.figure(figsize=(7, 3.5))
    f.subplots_adjust(left=None, bottom=0.16, right=None, top=None,
                    wspace=None, hspace=None)
    plt.plot(r, train_xy_errs, 'k--', label='training error', linewidth=2.0)
    plt.plot(r, test_xy_errs, 'k-', label='cross-val error', linewidth=2.0)
    plt.legend()
    plt.xlabel('training set size')
    plt.ylabel('normalized error')
    #plt.ylabel('rms error / avg output magnitude')
    plt.show()

if __name__=='__main__':
    main(sys.argv)



