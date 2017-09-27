#!/usr/bin/env python

import optparse, sys
import h5py
import config.helper as helper
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pdb

def plot(opt, args):
    if len(args) < 1:  # no bagfile name
        parser.error("Usage: plot_friction_map_fine.py [bag_file_path.h5]")
        return
    
    hdf5_filepath = args[0]
    figfname_png = hdf5_filepath.replace('.h5', '_fmap.png')
    figfname_pdf = hdf5_filepath.replace('.h5', '_fmap.pdf')
    figfname_std = hdf5_filepath.replace('.h5', '_fmapstd.png')
    figfname_hist_png = hdf5_filepath.replace('.h5', '_fmaphist.png')
    figfname_hist_pdf = hdf5_filepath.replace('.h5', '_fmaphist.pdf')
    ft_wrench = []
    
    min_x, max_x, min_y, max_y = opt.limits
    min_y += opt.res[1]*2   
    max_y -= opt.res[1]*2   
    rangex = np.arange(min_x, max_x, opt.res[0])
    rangey = np.arange(min_y, max_y, opt.res[1])
    n_x = len(rangex)
    n_y = len(rangey)
    
    f = h5py.File(hdf5_filepath, "r")
    tip_array = f['tip_array'].value
    ft_wrench = f['ft_wrench'].value
            
    image_vals = [[[] for i in range(n_y)] for j in range(n_x)]
    image_avg =  [[0 for i in range(n_y)] for j in range(n_x)]
    image_std =  [[0 for i in range(n_y)] for j in range(n_x)]
    all_vals = []
    all_image_vals = []
    
    N = opt.N
    
    scale = (len(ft_wrench)*1.0/len(tip_array))
    for i in range(len(tip_array)):
        ind = (int((tip_array[i][1]-min_x)/opt.res[0]), int((tip_array[i][2]-min_y)/opt.res[1]))

        
        ft_i = int(i * scale)
        if ind and ind[0]>=0 and ind[1]>=0 and ind[0] < n_x and ind[1] < n_y:
            image_vals[ind[0]][ind[1]].append(np.fabs(ft_wrench[ft_i][2]))  # force_y
            all_vals.append(np.fabs(ft_wrench[ft_i][2]))
        
    for idx in range(n_x):
        for idy in range(n_y):
            #print idx, idy, len(image_vals[idx][idy])
            if len(image_vals[idx][idy]) > 0:
                image_avg[idx][idy] = sum(image_vals[idx][idy]) / len(image_vals[idx][idy]) / N
                image_std[idx][idy] = np.std(image_vals[idx][idy]) / N
                all_image_vals.append(image_avg[idx][idy])
            else:
                image_avg[idx][idy] = image_avg[idx-1][idy] # hack
    
    print 'average', '%.3f' % (np.average(all_vals) / N)
    print 'std', '%.3f' % (np.std(all_vals) / N)
    print 'max', '%.3f' % (np.max(all_vals) / N)
    print 'min', '%.3f' % (np.min(all_vals) / N)
    minn = np.min(all_image_vals)
    maxx = np.max(all_image_vals)
    
    fig, ax = plt.subplots()
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif', size=30)
    plt.title(opt.title)
    #ax.set_title(opt.title)
    
    xticks = plt.xticks()
    yticks = plt.yticks()
    #print xticks
    plt.xticks(np.linspace(0, 0.6, 3))
    plt.yticks(np.linspace(0, 25, 3))
    #plt.yticks(np.linspace(yticks[0][1], yticks[0][-2], 3))
    
    plt.savefig(figfname_hist_png)
    plt.savefig(figfname_hist_pdf)
    plt.close()
    
    #print image_avg
    #print image_std
    fig, ax = plt.subplots()
    plt.rc('font', family='serif', size=30)
    plt.title(opt.title)
    plt.imshow(image_avg, extent=(rangey[0], rangey[-1]+opt.res[1], rangex[-1]+opt.res[0], rangex[0]),
           interpolation='nearest', cmap=cm.Greys, vmin=opt.avgcolorbar[0], vmax=opt.avgcolorbar[1])
    plt.gca().xaxis.set_visible(False)
    plt.gca().yaxis.set_visible(False)
    
    cbar = plt.colorbar(orientation='horizontal', ticks=[minn, maxx])
    cbar.ax.set_xticklabels([('%.2f' % minn), ('%.2f' % maxx)])
    
    plt.savefig(figfname_png)
    plt.savefig(figfname_pdf)
    
    print figfname_png
    
    #plt.show()
    plt.close()
    
    plt.imshow(image_std, extent=(rangey[0], rangey[-1]+opt.res[1], rangex[-1]+opt.res[0], rangex[0]),
           interpolation='nearest', cmap=cm.Greys)
           
    plt.colorbar(orientation='horizontal')
    
    plt.savefig(figfname_std)
    

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('', '--avgcolorbar', action="store", type='float', dest='avgcolorbar', 
                      help='Color bar', nargs=2, default=(None,None))
    parser.add_option('', '--res', action="store", type='float', dest='res', 
                      help='Resolution in meter', nargs=2, default=(0.005,0.005))
    parser.add_option('', '--limits', action="store", type='float', dest='limits', 
                      help='Limits [minx, maxx, miny, maxy]', nargs=4, default=(0.250,0.450, -0.233, 0.197)) 
    parser.add_option('', '--N', action="store", type='float', dest='N', 
                      help='Normal force', nargs=1, default=(0.8374 * 9.81)) 
                      
    parser.add_option('', '--title', action="store", type='string', dest='title', 
                      help='Title', default='')
                      
    (opt, args) = parser.parse_args()
    plot(opt, args)
    
    

if __name__=='__main__':
    main(sys.argv)
