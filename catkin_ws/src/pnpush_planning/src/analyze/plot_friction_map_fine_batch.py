#!/usr/bin/env python

import subprocess
import sys, os
import glob
import optparse
import plot_friction_map_fine

ss = ['abs', 'delrin', 'plywood', 'pu']
dirpath = '/home/mcube/pnpushdata/friction_scan_fine'
for s in ss:
    hdf5_filepath = '%s/%s/rect1/record_surface=%s_shape=rect1_a=0_v=20_rep=000.h5' % (dirpath, s, s)
    proc = subprocess.Popen('rosrun pnpush_planning plot_friction_map_fine.py %s --title %s' % 
              (hdf5_filepath, s) , shell=True)
    proc.wait()
    
