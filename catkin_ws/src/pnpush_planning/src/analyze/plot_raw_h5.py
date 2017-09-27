#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# Plot the trajectory

import numpy as np
import json
import h5py

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

from config.shape_db import ShapeDB

import tf.transformations as tfm
from ik.helper import *
from config.helper import *
from matplotlib.pyplot import savefig


def plot(data, shape_id, figfname):
    #data['tip_poses']
    #data['ft_wrench']
    #data['object_pose']
    
    probe_radii = {'probe1' : 0.00626/2, 'probe2': 0.004745, 'probe3': 0.00475, 'probe4': 0.00475}
    probe_radius = probe_radii['probe4']
    
    fig, ax = plt.subplots()
    fig.set_size_inches(7,7)
    
    v = int(getfield_from_filename(os.path.basename(figfname), 'v'))
    try:
        a = int(getfield_from_filename(os.path.basename(figfname), 'a'))
    except:
        a = 0
    
    if a!=0:
        sub = int((2500.0**2) * 2 /(a**2))
        if sub < 1: sub = 1
    elif v!=-1:
        sub = int(30*20 / (v))                 # subsample rate
        if sub < 1: sub = 1
    tip_pose = data['tip_pose']
    
    patches = []
    
    # add the object as polygon
    shape_db = ShapeDB()
    shape = shape_db.shape_db[shape_id]['shape'] # shape of the objects presented as polygon.
    shape_type = shape_db.shape_db[shape_id]['shape_type']
    if shape_type == 'poly':
        shape_polygon_3d = np.hstack((np.array(shape), np.zeros((len(shape), 1)), np.ones((len(shape), 1))))
    elif shape_type == 'ellip':
        shape = shape[0]
    elif shape_type == 'polyapprox':
        shape_polygon_3d = np.hstack((np.array(shape[0]), np.zeros((len(shape[0]), 1)), np.ones((len(shape[0]), 1))))
    
    object_pose = data['object_pose']
    
    if len(object_pose) > 0:
        #invT0 = np.linalg.inv(matrix_from_xyzquat(object_pose[0][1:4], object_pose[0][4:8]))
        invT0 = np.linalg.inv(matrix_from_xyzrpy(object_pose[0][1:3].tolist() + [0], [0,0,object_pose[0][3]]))
    elif len(tip_pose) > 0:
        invT0 = np.linalg.inv(matrix_from_xyzquat(tip_pose[0][1:3].tolist() +[0], [0,0,0,1]))


    print 'object_pose', len(object_pose), 'tip_pose', len(tip_pose)

    r = []
    if len(object_pose) > 0:
        r = (range(0, len(object_pose), sub)) + [len(object_pose)-1]
    for i in r:
        
        T = matrix_from_xyzrpy(object_pose[i][1:3].tolist() + [0], [0,0,object_pose[i][3]])
        
        if i == 0:
            alpha , fill = (0.3, True)
        elif i == r[-1]:
            alpha , fill = (0.6, True)
        else:
            alpha , fill = (0.6, False)
        
        ec, fc = 'black','orangered'
        if shape_type == 'poly' or shape_type == 'polyapprox':
            shape_polygon_3d_world = np.dot(np.dot(invT0, T), shape_polygon_3d.T)
            obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
        elif shape_type == 'ellip':
            T_T0 = np.dot(invT0, T)
            scale, shear, angles, trans, persp = tfm.decompose_matrix(T_T0)
            obj = mpatches.Ellipse(trans[0:2], shape[0]*2, shape[1]*2, angle=angles[2], fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
            
        ax.add_patch(obj)
    
    # add the probes as circle
    r = []
    if len(tip_pose) > 0:
        r = (range(0, len(tip_pose), sub)) + [len(tip_pose)-1]
    for i in r:
        tip_pose_0 = np.dot(invT0, tip_pose[i][1:3].tolist()+[0,1])
        if i == 0:
            alpha , fill = (0.8, False)
        elif i == r[-1]:
            alpha , fill = (0.8, False)
        else:
            alpha , fill = (0.8, False)
        circle = mpatches.Circle(tip_pose_0[0:2], probe_radius, color='black', alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
            
        ax.add_patch(circle)

    # render it
    plt.axis([-0.15, 0.15, -0.15, 0.15])
    plt.axis('off')
    
    if figfname is not None:
        plt.savefig(figfname)

def plot_force_profile(data, shape_id, figfname, multidim):
    object_pose = data['object_pose']
    invrotT0 = np.linalg.inv(rotmatrix_from_quat(object_pose[0][4:8]))
    
    
    # transform to object frame
    force_obj = [np.dot(invrotT0, np.array(ft_wrench[1:4] + [1])).tolist() for ft_wrench in data['ft_wrench']]
    
    multidim = True
    starttime = data['ft_wrench'][0][0]
    timearray = np.array(data['ft_wrench'])[:,0] - starttime
    if multidim:
        f, axarr = plt.subplots(3, sharex = True)
        axarr[0].plot(timearray, np.array(force_obj)[:,0])
        axarr[1].plot(timearray, np.array(force_obj)[:,1])
        axarr[2].plot(timearray, np.array(force_obj)[:,2])
        axarr[0].set_xlabel('time (sec)')
        axarr[0].set_ylabel('force (N)')
        
    else:
        f, ax = plt.subplots(1, sharex = True)
        ax.plot(np.array(timearray, np.array(force_obj)[:,1]))
        ax.set_xlabel('time (sec)')
        ax.set_ylabel('force (N)')
    plt.show()
    
def plot_speed_profile(data, shape_id, figfname, multidim):
    tip_pose = data['tip_pose']
    
    
    # transform to object frame
    #force_obj = [np.dot(invrotT0, np.array(ft_wrench[1:4] + [1])).tolist() for ft_wrench in data['ft_wrench']]
    #force_obj = [np.dot(invrotT0, np.array([1,2,3,4])).tolist() for ft_wrench in data['ft_wrench']]
    
    starttime = tip_pose[0][0]
    timearray = np.array(tip_pose)[:,0] - starttime
    
    tip_pose = np.array(tip_pose)
    timediff = (tip_pose[1:,0:1] - tip_pose[0:-1,0:1])
    
    tip_vel = (tip_pose[1:,1:3] - tip_pose[0:-1,1:3]) / np.hstack((timediff, timediff))
    tip_speed = np.sqrt(tip_vel[:,0:1]**2 + tip_vel[:,1:2]**2)
    
    f, ax = plt.subplots(1, sharex = True)
    ax.plot(timearray[1:], tip_speed)
    ax.set_xlabel('time (sec)')
    ax.set_ylabel('speed (m/s)')
    plt.show()



def main(argv):
    if len(argv) < 2:
        print 'Usage: plot_raw_json.py *.h5 tip_speed_profile/forceprofile/snapshots'
        return
    
    h5_filepath = argv[1]
    
    if len(argv) < 3:
        choice = 'snapshots'
    else:
        choice = argv[2]
        
    data = h5py.File(h5_filepath, "r", driver='core')
    
    
    figname = h5_filepath.replace('.h5', '.png')
    shape_id = getfield_from_filename(figname, 'shape')
    if shape_id == 'butt': 
        shape_id = 'butter'
    if choice == 'snapshots':
        plot(data, shape_id, figname)
    elif choice == 'forceprofile':
        plot_force_profile(data, shape_id, figname, multidim = True)
    elif choice == 'tip_speed_profile':
        plot_speed_profile(data, shape_id, figname, multidim = True)
        
    data.close()

if __name__=='__main__':
    import sys
    main(sys.argv)
    
