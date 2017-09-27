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

def plot_force_profile(data, mass, sec_start, sec_end):
    ft = np.array(data['ft_wrench'])
    start_time = ft[0,0]
    
    time_array = ft[:,0] - start_time
    cropped_ft = ft[(time_array < sec_end) & (time_array > sec_start), 1:4]
    cropped_time_array = time_array[(time_array < sec_end) & (time_array > sec_start)]
    
    norm_cropped_ft = [np.sqrt(cropped_ft[i,0]**2 + cropped_ft[i,1]**2 + cropped_ft[i,2]**2) for i in range(cropped_ft.shape[0])] 
    
    
    multidim = False
    if multidim:
        f, axarr = plt.subplots(3, sharex = True)
        axarr[0].plot(cropped_time_array, np.array(cropped_ft)[:,0])
        axarr[1].plot(cropped_time_array, np.array(cropped_ft)[:,1])
        axarr[2].plot(cropped_time_array, np.array(cropped_ft)[:,2])
        axarr[0].set_xlabel('time (sec)')
        axarr[0].set_ylabel('force (N)')
        
    else:
        f, ax = plt.subplots(1, sharex = True)
        plt.plot(cropped_time_array, norm_cropped_ft)
        ax.set_xlabel('time (sec)')
        ax.set_ylabel('force (N)')
        
    plt.show()
    
    g = 9.81
    coeff = np.average(np.array(norm_cropped_ft) / (mass * g))
    print coeff
    

def main(argv):
    if len(argv) < 3:
        print 'Usage: get_friction_coefficient.py *.json mass [sec_start] [sec_end]'
        return
    
    filepath = argv[1]
    
    mass = float(argv[2])
    
    if len(argv) == 5:
        sec_start, sec_end = (float(argv[3]), float(argv[4]))
    else:
        sec_start, sec_end = (0, np.inf)
    
    with open(filepath) as data_file:    
        data = json.load(data_file)
        
    plot_force_profile(data, mass, sec_start, sec_end)
    
if __name__=='__main__':
    import sys
    main(sys.argv)
