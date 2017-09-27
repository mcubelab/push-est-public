#!/usr/bin/env python

# Peter KT Yu, Jan 2017
# Extract the data

import numpy as np
import json, time, scipy.io
import pandas as pd

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.pyplot import savefig
from mpl_toolkits.mplot3d import Axes3D

from config.shape_db import ShapeDB
import transformations as tfm
from ik.helper import transform_pose_to_frame2d
from ik.helper import *
from config.helper import *
import random

rate = '400ms'
def extract2d_and_cleanup(data):
    tip_poses = np.array(data['all_contact'])[:,13:15]
    object_poses = np.array(data['all_contact'])[:,6:13]
    fts = np.array(data['all_contact'])[:,3:5]
    
    # transformation to the first object pose using 2d 3x3 homogeneous matrix
    # so that we don't need to deal with theta at pi or -pi?
    
    # object
    object_pose0 = xyth_from_xyzqwxyz(object_poses[0])
    
    object_pose_2d = []
    for object_pose in object_poses:
        tmp = transform_pose_to_frame2d(xyth_from_xyzqwxyz(object_pose), object_pose0)
        object_pose_2d.append(tmp)

    # probe
    tip_pose_2d = []
    for tip_pose in tip_poses:
        tmp = transform_pose_to_frame2d(list(tip_pose[0:2])+[0.0], object_pose0)
        tip_pose_2d.append(tmp)
        

    # ft
    ft_wrench_2d = fts   # only need the force, # todo this also need to rotate
    print 'object_pose_2d', len(object_pose_2d), 'tip_pose_2d', len(tip_pose_2d), 'ft_wrench_2d', len(ft_wrench_2d)
    
    data2d = {}
    data2d['tip_pose_2d'] = tip_pose_2d
    data2d['object_pose_2d'] = object_pose_2d
    data2d['ft_wrench_2d'] = ft_wrench_2d
    return data2d


def wraptopi(data):
    return ( data + np.pi) % (2 * np.pi ) - np.pi 


# data: synced 2d data
def extract_training_data(data):    
    tip_pose = data['tip_pose_2d']
    object_pose = data['object_pose_2d']
    force = data['ft_wrench_2d']
        
    # find the beginning of contact by force magnitude
    threshold = 0.5
    start_index = 0
    end_index = len(force)-1
    if rate == '400ms':
        sub = 20
    if rate == '100ms':
        sub = 5
    elif rate == '20ms':
        sub = 1
    print 'start_index', start_index, 'end_index', end_index, 'len', len(force)
    
    data_training = []  
    labels = ['$x$', '$y$', '$\Delta x$', '$\Delta y$', 'force $x$', 'force $y$', 
        r'$\Delta x$', r'$\Delta y$', r'$\Delta \theta$']
    
    for i in xrange(start_index, end_index+1-sub, sub):
        tip_pose_i_obji = transform_to_frame2d(tip_pose[i][0:2], object_pose[i])
        tip_pose_isub_obji = transform_to_frame2d(tip_pose[i+sub][0:2], object_pose[i])
        
        force_i_obji = rotate_to_frame2d(force[i], object_pose[i])
        
        object_pose_i_obji = transform_to_frame2d(object_pose[i][0:2], object_pose[i])
        object_pose_isub_obji = transform_to_frame2d(object_pose[i+sub][0:2], object_pose[i])
                
        newdata = tip_pose_i_obji + (np.array(tip_pose_isub_obji) - np.array(tip_pose_i_obji)).tolist() +\
        force_i_obji + (np.array(object_pose_isub_obji) - np.array(object_pose_i_obji)).tolist() + \
        [object_pose[i+sub][2] - object_pose[i][2]]
        
        #print 'push length' , norm(np.array(tip_pose_isub_obji) - np.array(tip_pose_i_obji))
        #print newdata
        data_training.append(newdata)
    
    
    return data_training

def prepare_and_save_mat(data_training, outputfile):
    
    # perpare for the matlab training data, [c, beta]
    input_training = []
    input_testing = []    
    output_training = []
    output_testing = []
    
    ntraining = 200
    ratio_train = float(ntraining) / len(data_training)
    for d in data_training:
        x = (d[0], d[1])
        vel = (d[2], d[3])
        motion = (d[6], d[7], d[8])
        
        # find which side
        if x[0] + x[1] >= 0 and x[0] - x[1] <= 0:
            sth = -np.pi/2; 
        elif x[0] + x[1] <= 0 and x[0] - x[1] <= 0:
            sth = 0; 
        elif x[0] + x[1] <= 0 and x[0] - x[1] >= 0:
            sth = np.pi/2; 
        elif x[0] + x[1] >= 0 and x[0] - x[1] >= 0:
            sth = np.pi; 
        else:
            print 'bad'
            import pdb; pdb.set_trace()
        
        
        # find beta from vel
        vel_side0 = rotate_to_frame2d(vel, [0,0,sth])     # convert it to side 0
        beta = wraptopi(np.arctan2(vel_side0[1], vel_side0[0])) 
        
        if beta > np.pi/2 or beta < -np.pi/2: 
            if beta > np.pi/2:
                sth += np.pi/2
            if beta < -np.pi/2: 
                sth -= np.pi/2
                
            # find beta again
            vel_side0 = rotate_to_frame2d(vel, [0,0,sth])     # convert it to side 0
            beta = wraptopi(np.arctan2(vel_side0[1], vel_side0[0])) 

        
        # find c
        x_side0 = rotate_to_frame2d(x, [0,0,sth])      # convert it to side 0
        c = (x_side0[1] + 0.045) / 0.09  # todo: need to be moved out
        c = max( min(1, c), 0)  # cap in [0,1]
        
        
        if beta > np.pi/2 or beta < -np.pi/2:
            print 'beta', beta, 'sth', sth
            import pdb; pdb.set_trace()
        
        
        
        motion_side0 = rotate_to_frame2d(motion[0:2], [0,0,sth])
        motion_side0 = motion_side0 + [wraptopi(motion[2])]
        
        if c >= 0.98 and abs(beta) <0.01:
            print 'c', c, 'beta', beta, 'motion_side0', motion_side0
            #import pdb; pdb.set_trace()
        #if c < 0.01 or c > 0.98:
        #    continue  # hack
            
        if abs(motion_side0[2]) > 1:
            continue;  # skip it  # need to know why
        
        if random.random() < ratio_train and len(input_training) < len(data_training) * ratio_train:
            input_training.append([c, beta])
            output_training.append(motion_side0)
        else:
            input_testing.append([c, beta])
            output_testing.append(motion_side0)
    scipy.io.savemat(outputfile, mdict={'input_training': input_training, 
                                        'output_training': output_training,
                                        'input_test': input_testing,
                                        'output_test': output_testing})

def json2trainingdata(filepath):
    # load the training data
    # data['tip_pose'] = [[timestamp, x, y, theta]]
    # data['object_pose'] = [[timestamp, x, y, theta]]
    # data['ft_wrench'] = [[timestamp, fx, fy, tz]]
    with open(filepath) as data_file:    
        data = json.load(data_file)
    
    shape_id = 'rect1'
    v = getfield_from_filename(filepath, 'v')
    data_synced = extract2d_and_cleanup(data)
    
    data_training = extract_training_data(data_synced)
    #data_training = extend_with_speedset(data_training, int(v))
    #plot_training_data(data_training)
    return data_training


def main(argv):
    import glob
    filelist = glob.glob("%s/all_contact*.json" % argv[1])
    all_training_data = []
    
    #filelist = ['all_contact_real_shape=rect1_rep=0000.json']
    for json_filepath in filelist[0:1]:
        print json_filepath
        all_training_data.extend(json2trainingdata(json_filepath))
        print 'len(all_training_data)', len(all_training_data)
    
    outputfile_mat= "%s/data_training_vhgp_%s.mat" % (argv[1], rate)
    prepare_and_save_mat(all_training_data, outputfile_mat)
    
    outputfile= "%s/data_training_vhgp_%s.json" % (argv[1], rate)
    with open(outputfile, 'w') as outfile:
        json.dump(all_training_data, outfile, indent=4)

if __name__=='__main__':
    import sys
    main(sys.argv)
    
