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

rate = 400
resample_rate = 4
def extract2d_and_cleanup(data):
    tip_poses = data['tip_pose']
    object_poses = data['object_pose']
    fts = data['ft_wrench']
    
    # transformation to the first object pose using 2d 3x3 homogeneous matrix
    # so that we don't need to deal with theta at pi or -pi?
    
    # object
    object_pose0 = object_poses[0][1:4]
    
    object_pose_2d = []
    for object_pose in object_poses:
        tmp = transform_pose_to_frame2d(object_pose[1:4], object_pose0)
        time = object_pose[0]
        if not (len(object_pose_2d) > 0 and time == object_pose_2d[-1][0] ) :
            object_pose_2d.append([time] + tmp)
        # check for redundancy, should be deleted

    # probe
    tip_pose_2d = []
    for tip_pose in tip_poses:
        tmp = transform_pose_to_frame2d(tip_pose[1:4], object_pose0)
        time = tip_pose[0]
        # check for redundancy, should be deleted
        if not(len(tip_pose_2d) > 0 and time == tip_pose_2d[-1][0] ):
            tip_pose_2d.append([time] + tmp)
        

    # ft
    ft_wrench_2d = np.array(fts)[:,0:3].tolist()   # only need the force, # todo this also need to rotate
    print 'object_pose_2d', len(object_pose_2d), 'tip_pose_2d', len(tip_pose_2d), 'ft_wrench_2d', len(ft_wrench_2d)
    
    data2d = {}
    data2d['tip_pose_2d'] = tip_pose_2d
    data2d['object_pose_2d'] = object_pose_2d
    data2d['ft_wrench_2d'] = ft_wrench_2d
    return data2d

def resample_using_pandas(data):
    force_2d = data['ft_wrench_2d']
    object_pose_2d = data['object_pose_2d']
    tip_pose_2d = data['tip_pose_2d']
    
    starttime = max(tip_pose_2d[0][0], object_pose_2d[0][0], force_2d[0][0])
    endtime = min(tip_pose_2d[-1][0], object_pose_2d[-1][0], force_2d[-1][0])
    
    pd_starttime = pd.to_datetime(starttime, unit='s')
    pd_endtime = pd.to_datetime(endtime, unit='s')
    
    resample_rate_str = '%dms' % resample_rate
    
    # tip poses
    tip_pose_2d_dt = pd.to_datetime(np.array(tip_pose_2d)[:,0].tolist(), unit='s')    
    tip_pose_2d = pd.DataFrame(np.array(tip_pose_2d)[:,1:4].tolist(), index=tip_pose_2d_dt)
    tip_pose_2d_resampled = tip_pose_2d.resample(resample_rate_str).mean()
    tip_pose_2d_interp = tip_pose_2d_resampled.interpolate()
    
    start_ = tip_pose_2d_interp.index.searchsorted(pd_starttime)
    end_ = tip_pose_2d_interp.index.searchsorted(pd_endtime)
    tip_pose_2d_interp = tip_pose_2d_interp.ix[start_:end_]
    tip_pose_2d_interp_list = tip_pose_2d_interp.values.tolist()
    
    # object poses
    object_pose_2d_dt = pd.to_datetime(np.array(object_pose_2d)[:,0].tolist(), unit='s')
    object_pose_2d = pd.DataFrame(np.array(object_pose_2d)[:,1:4].tolist(), index=object_pose_2d_dt)
    object_pose_2d_resampled = object_pose_2d.resample(resample_rate_str).mean()
    object_pose_2d_interp = object_pose_2d_resampled.interpolate()
    start_ = object_pose_2d_interp.index.searchsorted(pd_starttime)
    end_ = object_pose_2d_interp.index.searchsorted(pd_endtime)
    object_pose_2d_interp = object_pose_2d_interp.ix[start_:end_]
    object_pose_2d_interp_list = object_pose_2d_interp.values.tolist()
    
    force_dt = pd.to_datetime(np.array(force_2d)[:,0].tolist(), unit='s')
    force_2d = pd.DataFrame(np.array(force_2d)[:,1:3].tolist(), index=force_dt)   # only extract force
    force_2d_resampled = force_2d.resample(resample_rate_str).mean()
    force_2d_interp = force_2d_resampled.interpolate()
    start_ = force_2d_interp.index.searchsorted(pd_starttime)
    end_ = force_2d_interp.index.searchsorted(pd_endtime)
    force_2d_interp = force_2d_interp.ix[start_:end_]
    force_2d_interp_list = force_2d_interp.values.tolist()
    
    
    data_resample = {}
    data_resample['tip_pose_2d'] = tip_pose_2d_interp_list
    data_resample['object_pose_2d'] = object_pose_2d_interp_list
    data_resample['ft_wrench_2d'] = force_2d_interp_list
    return data_resample
    
def animate_2dsynced(data, shape_id, figfname):
    fig, ax = plt.subplots()
    probe_radius = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    
    v = int(figfname.split('_')[-4].split('=')[1])
    sub = 1                 # subsample rate
    tip_pose = data['tip_poses_2d']
    object_pose = data['object_poses_2d']
    force = data['force_2d']
    
    patches = []
    
    
    # add the object as polygon
    shape_db = ShapeDB()
    shape_polygon = shape_db.shape_db[shape_id]['shape_poly'] # shape of the objects presented as polygon.
    shape_polygon_3d = np.hstack((np.array(shape_polygon), np.zeros((len(shape_polygon), 1)), np.ones((len(shape_polygon), 1))))
    

    print 'object_pose', len(object_pose), 'tip_pose', len(tip_pose), 'force', len(force)
    plt.ion()
    for i in (range(0, len(tip_pose), sub)):
        
        plt.cla()
        T = tfm.compose_matrix(translate = object_pose[i][0:2] + [0], angles = (0,0,object_pose[i][2]) )
        shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
        
        obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, color='blue', alpha=0.05)
        ax.add_patch(obj)
    
        # add the probes as circle
        circle = mpatches.Circle(tip_pose[i][0:2], probe_radius, ec="none", color='red', alpha=0.5)
        ax.add_patch(circle)
        
        # add the force
        ax.arrow(tip_pose[i][0], tip_pose[i][1], force[i][0]/100, force[i][1]/100, head_width=0.005, head_length=0.01, fc='k', ec='k')
        
        #arrow = mpatches.Arrow(tip_pose[i][0], tip_pose[i][1], force[i][0],
        #                force[i][1], head_width=0.05, head_length=0.1, fc='k', ec='k')
        #ax.add_patch(arrow)
        
        # render it
        plt.axis([-0.1, 0.1, -0.1, 0.1])
        #plt.axis('equal')
        plt.draw()
        #time.sleep(0.1)
    plt.show()



# data: synced 2d data
def extract_training_data(data):   
    global rate 
    tip_pose = data['tip_pose_2d']
    object_pose = data['object_pose_2d']
    force = data['ft_wrench_2d']
        
    # find the beginning of contact by force magnitude
    threshold = 0.5
    start_index = 0
    end_index = 0
    sub = int(rate / resample_rate)
    for i in xrange(0, len(force)-1, 1):
        if norm(np.array(tip_pose[i][0:2]) - np.array(tip_pose[i+1][0:2]))>0 and \
           norm(np.array(object_pose[i]) - np.array(object_pose[i+1]))>1e-3 and \
           norm(force[i]) > threshold:
            start_index = i+2
            break
            
    for i in xrange(len(force)-2, -1, -1):
        if norm(np.array(tip_pose[i][0:2]) - np.array(tip_pose[i+1][0:2]))>0 and \
           norm(np.array(object_pose[i]) - np.array(object_pose[i+1]))>1e-3 and \
           norm(force[i]) > threshold:
            end_index = i-2
            break
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

def extend_with_speedset(data, v):
    return [d + [v] for d in data]

def prepare_and_save_mat(data_training, outputfile):
    
    # perpare for the matlab training data, [c, beta]
    input_training = []
    input_testing = []    
    output_training = []
    output_testing = []
    
    ntraining = 500*2
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
            
        if abs(motion_side0[2]) > 1:
            continue;  # skip it  # need to know why
        
        if random.random() < float(ntraining) / len(data_training) and len(input_training) < ntraining:
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
    data2d = extract2d_and_cleanup(data)
    data_synced = resample_using_pandas(data2d)
    #animate_2dsynced(data_synced, shape_id, filepath.replace('.json', '.png'))
    #data_synced = extend_with_velocity(data_synced, h=0.02)
    
    data_training = extract_training_data(data_synced)
    #data_training = extend_with_speedset(data_training, int(v))
    #plot_training_data(data_training)
    return data_training


def main(argv):
    import glob
    global rate
    filelist = glob.glob("%s/motion*.json" % argv[1])
    rate = int(argv[2])
    all_training_data = []
    
    for json_filepath in filelist:
        # only process v=10
        if json_filepath.find('v=10_') == -1:
           continue
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
    
