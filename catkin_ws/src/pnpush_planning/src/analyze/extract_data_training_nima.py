#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# Extract the pushes into 2d and sync

import numpy as np
import json

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

from config.shape_db import ShapeDB

import tf.transformations as tfm
from ik.helper import *
from matplotlib.pyplot import savefig
import time

from mpl_toolkits.mplot3d import Axes3D

import pdb
def extract2d_and_cleanup(data):

    probe_radius = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    
    tip_pose = data['tip_poses']
    object_pose = data['object_pose']
    ft = data['ft_wrench']
    
    # transformation to the first 
    invT0 = np.linalg.inv(matrix_from_xyzquat(object_pose[0][1:4], object_pose[0][4:8]))
    sub = 1
    
    # object
    object_pose_2d = []
    for i in (range(0, len(object_pose), sub)):
        T = matrix_from_xyzquat(object_pose[i][1:4], object_pose[i][4:8])
        tip_pose_object0 = np.dot(invT0, T)
        scale, shear, angles, trans, persp = tfm.decompose_matrix(tip_pose_object0)
        #print 'trans', trans[0:2], 'angle', angles[2]
        time = object_pose[i][0]
        # don't add redundant data entry with the same time
        if(not(len(object_pose_2d) > 0 and time == object_pose_2d[-1][0] )):
            object_pose_2d.append([time] + trans[0:2].tolist() + [angles[2]])
    
    # probe
    tip_pose_2d = []
    for i in (range(0, len(tip_pose), sub)):
        tip_pose_0 = np.dot(invT0, tip_pose[i][1:4]+[1])
        #print 'trans', tip_pose_0[0:2]
        time = tip_pose[i][0]
        
        # don't add redundant data entry with the same time
        if(not(len(tip_pose_2d) > 0 and time == tip_pose_2d[-1][0] )):
            tip_pose_2d.append([time] + tip_pose_0[0:2].tolist())

    # ft, no redundency
    ft_2d = np.array(ft)[:,0:3].tolist()   # only need the force
    print 'object_pose_2d', len(object_pose_2d), 'tip_pose_2d', len(tip_pose_2d), 'ft_2d', len(ft_2d)
    
    data2d = {}
    data2d['tip_poses_2d'] = tip_pose_2d
    data2d['object_poses_2d'] = object_pose_2d
    data2d['force_2d'] = ft_2d
    return data2d


import pandas as pd

def resample_using_pandas(data):
    force_2d = data['force_2d']
    object_poses_2d = data['object_poses_2d']
    tip_poses_2d = data['tip_poses_2d']
    starttime = max(tip_poses_2d[0][0], object_poses_2d[0][0], force_2d[0][0])
    endtime = min(tip_poses_2d[-1][0], object_poses_2d[-1][0], force_2d[-1][0])
    
    pd_starttime = pd.to_datetime(starttime, unit='s')
    pd_endtime = pd.to_datetime(endtime, unit='s')
    
    tip_poses_2d_dt = pd.to_datetime(np.array(tip_poses_2d)[:,0].tolist(), unit='s')    
    tip_poses_2d = pd.DataFrame(np.array(tip_poses_2d)[:,1:3].tolist(), index=tip_poses_2d_dt)
    tip_poses_2d_resampled = tip_poses_2d.resample('10ms', how='mean')
    tip_poses_2d_interp = tip_poses_2d_resampled.interpolate()
    
    start_ = tip_poses_2d_interp.index.searchsorted(pd_starttime)
    end_ = tip_poses_2d_interp.index.searchsorted(pd_endtime)
    tip_poses_2d_interp = tip_poses_2d_interp.ix[start_:end_]
    tip_poses_2d_interp_list = tip_poses_2d_interp.values.tolist()
    
    object_poses_2d_dt = pd.to_datetime(np.array(object_poses_2d)[:,0].tolist(), unit='s')
    object_poses_2d = pd.DataFrame(np.array(object_poses_2d)[:,1:4].tolist(), index=object_poses_2d_dt)
    object_poses_2d_resampled = object_poses_2d.resample('10ms', how='mean')
    object_poses_2d_interp = object_poses_2d_resampled.interpolate()
    start_ = object_poses_2d_interp.index.searchsorted(pd_starttime)
    end_ = object_poses_2d_interp.index.searchsorted(pd_endtime)
    object_poses_2d_interp = object_poses_2d_interp.ix[start_:end_]
    object_poses_2d_interp_list = object_poses_2d_interp.values.tolist()
    
    force_dt = pd.to_datetime(np.array(force_2d)[:,0].tolist(), unit='s')
    force_2d = pd.DataFrame(np.array(force_2d)[:,1:3].tolist(), index=force_dt)
    force_2d_resampled = force_2d.resample('10ms', how='mean')
    force_2d_interp = force_2d_resampled.interpolate()
    start_ = force_2d_interp.index.searchsorted(pd_starttime)
    end_ = force_2d_interp.index.searchsorted(pd_endtime)
    force_2d_interp = force_2d_interp.ix[start_:end_]
    force_2d_interp_list = force_2d_interp.values.tolist()
    
    
    data_resample = {}
    data_resample['tip_poses_2d'] = tip_poses_2d_interp_list
    data_resample['object_poses_2d'] = object_poses_2d_interp_list
    data_resample['force_2d'] = force_2d_interp_list
    return data_resample
    
def animate_2dsynced(data, shape_id, figfname):
    fig, ax = plt.subplots()
    fig.set_size_inches((7,7))
    probe_radius = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    
    sub = 1                 # subsample rate
    tip_pose = data['tip_poses_2d']
    object_pose = data['object_poses_2d']
    force = data['force_2d']
    
    patches = []
    
    
    # add the object as polygon
    shape_db = ShapeDB()
    shape_polygon = shape_db.shape_db[shape_id]['shape'] # shape of the objects presented as polygon.
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
        plt.axis([-0.3, 0.3, -0.3, 0.3])
        #plt.axis('equal')
        plt.draw()
        #time.sleep(0.1)
    plt.show()


# data: synced 2d data
def extract_training_data(data):
    
    tip_pose = data['tip_poses_2d']
    object_pose = data['object_poses_2d']
    force = data['force_2d']
    
    
    # find the beginning of contact by force magnitude
    threshold = 0.5
    start_index = 0
    end_index = 0
    sub = 5
    for i in (range(0, len(force)-1, 1)):
        if norm(np.array(tip_pose[i]) - np.array(tip_pose[i+1]))>0 and norm(np.array(object_pose[i]) - np.array(object_pose[i+1]))>1e-3 and norm(force[i]) > threshold:
            start_index = i
            break
            
    for i in (range(len(force)-2, -1, -1)):
        if norm(np.array(tip_pose[i]) - np.array(tip_pose[i+1]))>0 and norm(np.array(object_pose[i]) - np.array(object_pose[i+1]))>1e-3 and norm(force[i]) > threshold:
            end_index = i
            break
    print 'start_index', start_index, 'end_index', end_index, 'len', len(force)
    
    data_training = []  # tip_x, tip_y, tip_vx, tip_vy, forcex, forcey, object_pose_vx, object_pose_vy, object_pose_vtheta
    
    for i in (range(start_index, end_index+1-sub, 1)):
        data_training.append(tip_pose[i]+force[i]+object_pose[i])
        
    return data_training

def animate_2dsynced2(data, shape_id, figfname):
    fig, ax = plt.subplots()
    fig.set_size_inches((7,7))
    probe_radius = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    
    sub = 1                 # subsample rate
    tip_pose = np.array(data)[:, 0:2].tolist()
    object_pose = np.array(data)[:, 4:7].tolist()
    force = np.array(data)[:, 2:4].tolist()
    
    patches = []
    
    
    # add the object as polygon
    shape_db = ShapeDB()
    shape_polygon = shape_db.shape_db[shape_id]['shape'] # shape of the objects presented as polygon.
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
        
        # render it
        plt.axis([-0.3, 0.3, -0.3, 0.3])
        #plt.axis('equal')
        plt.draw()
        #time.sleep(0.1)
    plt.show()

def json2trainingdata(filepath):
        
    with open(filepath) as data_file:    
        data = json.load(data_file)
    
    shape_id = 'rect1'
    data2d = extract2d_and_cleanup(data)
    data_synced = resample_using_pandas(data2d)
    #animate_2dsynced(data_synced, shape_id, filepath.replace('.json', '.png'))
    data_training = extract_training_data(data_synced)
    animate_2dsynced2(data_training, shape_id, filepath.replace('.json', '.png'))
    #plot_training_data(data_training)
    return data_training


def main(argv):
    import glob
    if len(argv) < 2:
        print 'Usage: extract_data_training_nima.py dirpath'
        return
        
    filelist = glob.glob("%s/*.json" % argv[1])
    all_training_data = []
    for json_filepath in filelist:
        if json_filepath.find('synced.json') >= 0:
            continue
        training_data = json2trainingdata(json_filepath)
        outputfile= json_filepath.replace('.json','_synced.json')
        with open(outputfile, 'w') as outfile:
            json.dump(training_data, outfile, indent=4)
    
    labels = ['tip_x', 'tip_y', 'tip_vx', 'tip_vy', 'forcex', 'forcey', 'object_pose_vx', 'object_pose_vy', 'object_pose_vtheta']
    

if __name__=='__main__':
    import sys
    main(sys.argv)
    
