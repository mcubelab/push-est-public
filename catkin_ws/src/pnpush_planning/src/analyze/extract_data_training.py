#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# Plot the trajectory

import numpy as np
import json

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

from config.shape_db import ShapeDB

import tf.transformations as tfm
from ik.helper import *
from config.helper import *
from matplotlib.pyplot import savefig
import time

from mpl_toolkits.mplot3d import Axes3D

import pdb
def extract2d_and_cleanup(data):
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

def wraptopi(data):
    return ( data + np.pi) % (2 * np.pi ) - np.pi 

def extend_with_velocity(data, h):
    h = 0.01  # from 10ms
    
    tip_pos = np.array(data['tip_poses_2d'])
    object_pos = np.array(data['object_poses_2d'])
    #nd = tip_pose.shape[0]
    
    tip_vel = (tip_pos[2:,:] - tip_pos[0:-2,:]) / 2 / h
    tip_pos = tip_pos[1:-1,:]
    
    object_vel_xy = (object_pos[2:,0:2] - object_pos[0,0:2]) / 2 / h
    object_vel_th = wraptopi(object_pos[2:,2:] - object_pos[0,2:]) / 2 / h
    object_pos = object_pos[1:-1,:]
    
    data_extended = {}
    data_extended['tip_poses_2d'] = tip_pos.tolist()
    data_extended['tip_vels_2d_vel'] = tip_vel.tolist()
    data_extended['object_poses_2d'] = object_pos.tolist()
    data_extended['object_vels_2d'] = np.hstack((object_vel_xy, object_vel_th)).tolist()
    data_extended['force_2d'] = data['force_2d'][1:-1]
    
    return data_extended


# data: synced 2d data
def extract_training_data(data):    
    tip_pose = data['tip_poses_2d']
    tip_vel = data['tip_vels_2d_vel']
    object_pose = data['object_poses_2d']
    object_vel = data['object_vels_2d']
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
    
    data_training = []  
    labels = ['$x$', '$y$', '$\Delta x$', '$\Delta y$', 'force $x$', 'force $y$', r'$\Delta x$', r'$\Delta y$', r'$\Delta \theta$',
        '$v_x$', '$v_y$', '$v_x$', '$v_y$',  # 9:  tip_svx, tip_svy, tip_evx, tip_evy, 
        '$v_x$', '$v_y$', r'$\omega$',       # 13: object_pose_svx, object_pose_svy, object_pose_svtheta, # start speed
        '$v_x$', '$v_y$', r'$\omega$']       # 16: object_pose_evx, object_pose_evy, object_pose_evtheta  # end speed]
                                             # 19: tip_speedset (mm)
    
    for i in (range(start_index, end_index+1-sub, sub)):
        tip_pose_i_obji = transform_to_frame2d(tip_pose[i], object_pose[i])
        tip_pose_isub_obji = transform_to_frame2d(tip_pose[i+sub], object_pose[i])
        
        tip_vel_i_obji = rotate_to_frame2d(tip_vel[i], object_pose[i])
        tip_vel_isub_obji = rotate_to_frame2d(tip_vel[i+sub], object_pose[i])
        
        force_i_obji = rotate_to_frame2d(force[i], object_pose[i])
        
        object_pose_i_obji = transform_to_frame2d(object_pose[i][0:2], object_pose[i])
        object_pose_isub_obji = transform_to_frame2d(object_pose[i+sub][0:2], object_pose[i])
        
        object_vel_i_obji = rotate_to_frame2d(object_vel[i][0:2], object_vel[i])
        object_vel_isub_obji = rotate_to_frame2d(object_vel[i+sub][0:2], object_vel[i])
        
        newdata = tip_pose_i_obji + (np.array(tip_pose_isub_obji) - np.array(tip_pose_i_obji)).tolist() +\
        force_i_obji + (np.array(object_pose_isub_obji) - np.array(object_pose_i_obji)).tolist() + \
        [object_pose[i+sub][2] - object_pose[i][2]] + \
        tip_vel_i_obji + tip_vel_isub_obji + \
        object_vel_i_obji + [object_vel[i][2]] + object_vel_isub_obji + [object_vel[i+sub][2]]
        
        #print newdata
        data_training.append(newdata)
        
    return data_training

def extend_with_speedset(data, v):
    return [d + [v] for d in data]

def json2trainingdata(filepath):
        
    with open(filepath) as data_file:    
        data = json.load(data_file)
    
    shape_id = 'rect1'
    v = getfield_from_filename(filepath, 'v')
    data2d = extract2d_and_cleanup(data)
    data_synced = resample_using_pandas(data2d)
    #animate_2dsynced(data_synced, shape_id, filepath.replace('.json', '.png'))
    data_synced = extend_with_velocity(data_synced, h=0.01)
    
    data_training = extract_training_data(data_synced)
    data_training = extend_with_speedset(data_training, int(v))
    #plot_training_data(data_training)
    return data_training


def main(argv):
    import glob
    filelist = glob.glob("%s/motion*.json" % argv[1])
    all_training_data = []
    
    for json_filepath in filelist:
        # only process v=20
        # if json_filepath.find('v=20_') == -1:
            # continue
            
        # try:
        all_training_data.extend(json2trainingdata(json_filepath))
        print 'len(all_training_data)', len(all_training_data)
        # except:
            # print json_filepath
            # pass
    
    
    outputfile= "%s/data_training_with_vel.json" % argv[1]
    with open(outputfile, 'w') as outfile:
        json.dump(all_training_data, outfile, indent=4)

if __name__=='__main__':
    import sys
    main(sys.argv)
    
