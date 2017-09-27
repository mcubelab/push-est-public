#!/usr/bin/env python

import optparse, sys
import h5py
import config.helper as helper
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('', '--avgcolorbar', action="store", type='float', dest='avgcolorbar', 
                      help='Color bar', nargs=2, default=(0,1))
                      
    (opt, args) = parser.parse_args()
    
    if len(args) < 1:  # no bagfile name
        parser.error("Usage: plot_friction_map.py [bag_file_path.h5]")
        return
    
    hdf5_filepath = args[0]
    figfname = hdf5_filepath.replace('.h5', '_fmap.png')
    figfname_std = hdf5_filepath.replace('.h5', '_fmapstd.png')
    ft_wrench = []
    
    max_x = 0.450
    min_x = 0.250
    rangex = [0.45, 0.35, 0.25]
    # 0.450, 0.350, 0.250
    max_y = 0.197-0.1
    min_y = -0.233+0.1
    rangey = [0.07, -0.03, -0.13]
    # -0.13, -0.03, 0.07, 
    
    f = h5py.File(hdf5_filepath, "r")
    tip_array = f['tip_array'].value
    ft_wrench = f['ft_wrench'].value
        
    radius = 0.01
    
    image_vals = [[[],[],[]],[[],[],[]],[[],[],[]]]
    image_avg = [[1,2,3],[0,0,0],[0,0,0]]
    image_std = [[0,0,0],[0,0,0],[0,0,0]]
    
    N = 0.8374 * 9.81
    
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
            print idx, idy, len(image_vals[idx][idy])
            image_avg[idx][idy] = sum(image_vals[idx][idy]) / len(image_vals[idx][idy]) / N
            image_std[idx][idy] = np.std(image_vals[idx][idy]) / N
    
    
    
    print image_avg
    print image_std
    
    plt.rc('font', family='serif', size=30)
    plt.imshow(image_avg, extent=(rangey[0]+0.05, rangey[2]-0.05, rangex[2]-0.05, rangex[0]+0.05),
           interpolation='nearest', cmap=cm.Greys, vmin=opt.avgcolorbar[0], vmax=opt.avgcolorbar[1])
           
    plt.colorbar()
    
    plt.savefig(figfname)
    
    print figfname
    
    plt.close()
    
    plt.imshow(image_std, extent=(rangey[0]+0.05, rangey[2]-0.05, rangex[2]-0.05, rangex[0]+0.05),
           interpolation='nearest', cmap=cm.Greys, vmin=0.01, vmax=0.03)
           
    plt.colorbar()
    
    plt.savefig(figfname_std)
    
    #plt.show()

if __name__=='__main__':
    main(sys.argv)
