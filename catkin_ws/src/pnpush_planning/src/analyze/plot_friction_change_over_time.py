#!/usr/bin/env python


import subprocess
import sys, os
import glob
import optparse
import matplotlib.pyplot as plt
import h5py
import config.helper as helper
import numpy as np
import pdb

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('', '--fmt', action="store", dest='fmt', 
                      help='Figure format e.g. png, pdf', default='png')
  
    (opt, args) = parser.parse_args()
    
    
    if len(args) < 2:
        parser.error("Usage: plot_friction_change_over_time.py [file_pattern.h5]")
        return
    
    filepattern = args[0]  # ./record_surface=plywood_shape=rect1_a=-1000_v=20_rep=%3d.h5
    filenum = int(args[1])  # 100
    
    figfname = filepattern.replace('.h5', '.%s' % opt.fmt)
    
    max_x = 0.450
    min_x = 0.250
    rangex = [0.45, 0.35, 0.25]
    max_y = 0.197-0.1
    min_y = -0.233+0.1
    rangey = [0.07, -0.03, -0.13]
    
    target = [0.35, -0.03]
    
    radius = 0.01
    
    image_avgs_t = []
    image_stds_t = []
    
    N = 0.8374 * 9.81
    nan = float('nan')
    if not os.path.exists(filepattern.replace('.h5', '_fcot.h5')):
        for f_ind in range(filenum):
            hdf5_filepath = filepattern % f_ind
            print 'processing', hdf5_filepath
            
            if not os.path.exists(hdf5_filepath):  # in case the data is bad
                print 'not found'
                image_avgs_t.append([[nan,nan,nan],[nan,nan,nan],[nan,nan,nan]])
                image_stds_t.append([[nan,nan,nan],[nan,nan,nan],[nan,nan,nan]])
                continue
                
            f = h5py.File(hdf5_filepath, "r")
            tip_array = f['tip_array'].value
            ft_wrench = f['ft_wrench'].value
            
            image_vals = [[[],[],[]],[[],[],[]],[[],[],[]]]
            image_avgs = [[0,0,0],[0,0,0],[0,0,0]]
            image_stds = [[0,0,0],[0,0,0],[0,0,0]]
            scale = (len(ft_wrench)*1.0/len(tip_array))
            for i in range(len(tip_array)):
                ind = None
                for idx in range(len(rangex)):
                    for idy in range(len(rangey)):
                        if helper.norm(np.array(tip_array[i][1:3]) - np.array([rangex[idx], rangey[idy]])) < radius:
                            ind = (idx,idy)
                            break
                    if ind is not None:
                        break
                ft_i = int(i * scale)
                if ind:
                    image_vals[ind[0]][ind[1]].append(np.fabs(ft_wrench[ft_i][2]))  # force_y
              
            for idx in range(len(rangex)):
                for idy in range(len(rangey)):
                    if len(image_vals[idx][idy]) > 0:
                        image_avgs[idx][idy] = sum(image_vals[idx][idy]) / len(image_vals[idx][idy]) / N
                    image_stds[idx][idy] = np.std(image_vals[idx][idy]) / N
            
                    
            image_avgs_t.append(image_avgs)
            image_stds_t.append(image_stds)
            
            f.close()
        with h5py.File(filepattern.replace('.h5', '_fcot.h5'), "w") as f:
            f.create_dataset("image_avgs_t", data=image_avgs_t)
            f.create_dataset("image_stds_t", data=image_stds_t)
    else:
        with h5py.File(filepattern.replace('.h5', '_fcot.h5'), "r") as f:
            image_avgs_t = f['image_avgs_t'].value
            image_stds_t = f['image_stds_t'].value
    
    # for idx in range(len(rangex)):
        # for idy in range(len(rangey)):
            # tmp = []
            # for t in range(filenum):
                # tmp.append(image_avgs_t[t][idx][idy])
            # plt.errorbar(range(1,filenum+1), tmp, fmt='-o', color='k')
            
    
    rangexx = []
    vals = []
    print len(image_avgs_t)
    for t in range(filenum):
        tmp = 0
        
        for idx in range(len(rangex)):
            for idy in range(len(rangey)):
                tmp += image_avgs_t[t][idx][idy] / 9.0
        
        if not np.isnan(tmp):
            vals.append(tmp)
            rangexx.append(t+1)
        
    plt.errorbar(rangexx, vals, fmt='-o', color='k')
    
    
    plt.ylabel('Coefficient of friction')
    plt.xlabel('Push time')
    #plt.title('Change of frictional coefficient over time')
    plt.savefig(figfname)
    
    plt.show()

if __name__=='__main__':
    main(sys.argv)
    
