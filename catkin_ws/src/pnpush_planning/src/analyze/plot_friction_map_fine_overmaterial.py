#!/usr/bin/env python

import subprocess
import sys, os
import glob
import optparse
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import h5py
import matplotlib.lines as mlines
from mpl_toolkits.axes_grid1 import make_axes_locatable
from latexify import latexify


def extract(opt, hdf5_filepath):
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
    
    return (all_vals, image_avg, (minn, maxx), (rangex, rangey))
    


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
    parser.add_option('', '--dirpath', action="store", type='string', dest='dirpath', 
                      help='Dirpath', default='./')
                      
    (opt, args) = parser.parse_args()
    
    cachefile = '/tmp/plot_friction_map_fine_overmaterial'
    import shelve
    if os.path.exists(cachefile):
        f = shelve.open(cachefile)
        ss = f['ss'];
        ncol = f['ncol'];
        all_valss = f['all_valss']
        image_avgs = f['image_avgs']
        image_range = f['image_range']
        image_ranges = f['image_ranges']
        opt = f['opt']
        rangex = f['rangex']
        rangey = f['rangey']
    else:
        ss = ['abs', 'delrin', 'plywood', 'pu']
        ncol = 2
        all_valss = []
        image_avgs = []
        image_range = [100, -100]
        image_ranges = []
        
        for i,s in enumerate(ss):
            hdf5_filepath = '%s/%s/rect1/record_surface=%s_shape=rect1_a=0_v=20_rep=000.h5' % (opt.dirpath, s, s)
            (all_vals, image_avg, (minn, maxx), (rangex, rangey)) = extract(opt, hdf5_filepath)
            all_valss.append(all_vals)
            image_avgs.append(image_avg)
            
            import scipy.io as sio
            mat_filepath = '%s/%s/rect1/friction_over_location_map_surface=%s.mat' % (opt.dirpath, s,s )
            import numpy as np
            sio.savemat(mat_filepath, {'image_avg':image_avg})
            
            image_ranges.append([minn, maxx])
            if minn < image_range[0]:
                image_range[0] = minn
            if maxx > image_range[1]:
                image_range[1] = maxx
                
        ll = locals()
        f = shelve.open(cachefile, 'n')
        for key, val in ll.iteritems():
            try:
                f[key] = val
            except:
                pass
    
    
    figfname_png = opt.dirpath + '/friction_over_location_map.png'
    figfname_pdf = opt.dirpath + '/friction_over_location_map.pdf'
    figfname_mat = opt.dirpath + '/friction_over_location_map.pdf'
    figfname_hist_png = opt.dirpath + '/friction_over_location_hist.png'
    figfname_hist_pdf = opt.dirpath + '/friction_over_location_hist.pdf'
    
    # plot the distribution
    
    #plt.rc('text', usetex=True)
    #fig = plt.figure(figsize=(8,4))
    #plt.rc('font', family='serif', size=15, serif=['Times'])
    latexify(scale = 2)
    #fig, ax = plt.subplots()
    ax = plt.gca()
    bins = np.linspace(0, 0.6, 500)
    colors = ['r', 'b', 'g', 'k']
    
    linestyles = [':', '-', '-.', '--']
    markerstyles = ['', '', '', '']
    
    legend_handles = []
    for i, s in enumerate(ss):
        print s
        #plt.hist(np.array(all_vals)/opt.N, bins, normed=1, histtype='stepfilled', facecolor='white', alpha=0.75)
        n, bins, patches= ax.hist(np.array(all_valss[i])/opt.N, bins, normed=1, histtype='stepfilled', 
                 facecolor='none', edgecolor=colors[i], label=s, alpha=0)
        
        #import pdb; pdb.set_trace()
        print 'mean:', np.mean(all_valss[i])/opt.N
        print 'std:', np.std(all_valss[i])/opt.N
        print 'std (%):', np.std(all_valss[i])/ np.mean(all_valss[i]) * 100
        
        bincenters = 0.5*(bins[1:]+bins[:-1])
        # add a 'best fit' line for the normal PDF
        #y = mlab.normpdf( bincenters, mu, sigma)
        #import pdb; pdb.set_trace()
        l, = ax.plot(bincenters, n, linestyles[i]+markerstyles[i], linewidth=2, markevery=20, label=s,
                     color=colors[i], markersize=4, markerfacecolor='white')
        # do legend
        #line = mlines.Line2D([], [], color=colors[i], label=s)
        legend_handles.append(l)
        
    #plt.xticks(np.linspace(0, 0.6, 3))
    #plt.yticks(np.linspace(0, 25, 3))
    plt.ylabel('Probability')
    plt.xlabel('Coefficient of friction')
    legend = ax.legend(legend_handles, ss)
    legend.get_frame().set_linewidth(0.25)

    
    ax.yaxis.set_visible(True)
    plt.tight_layout(pad=0.4)
    
    plt.savefig(figfname_hist_png)
    plt.savefig(figfname_hist_pdf)
    #plt.show()
    plt.close()
    
    
    latexify(scale = 1)
    fig, axes = plt.subplots((len(ss)+ncol-1)/ncol , ncol)
    #plt.rc('font', family='serif', size=15)
    
    ims = []
    for i, s in enumerate(ss):
        
        im = axes.flat[i].imshow(image_avgs[i], extent=(rangey[0], rangey[-1]+opt.res[1], rangex[-1]+opt.res[0], rangex[0]),
                             interpolation='nearest', cmap=cm.Greys, vmin=image_ranges[i][0], vmax=image_ranges[i][1])

        divider = make_axes_locatable(axes.flat[i])
        cax = divider.append_axes("bottom", size="5%", pad=0.05)

        cbar = plt.colorbar(im, cax=cax, orientation='horizontal', ticks=image_ranges[i])
        cbar.ax.set_xticklabels([('%.2f' % image_ranges[i][0]), ('%.2f' % image_ranges[i][1])])
        
        
        axes.flat[i].xaxis.set_visible(False)
        axes.flat[i].yaxis.set_visible(False)
        axes.flat[i].set_title(s)
        
    #plt.legend()
    
    # fig.subplots_adjust(left=0, bottom=0, right=1, top=0.95,
                    # wspace=0.18, hspace=0.17)
    plt.tight_layout(pad=0.4, w_pad=0, h_pad=1.0)
    # fig.subplots_adjust(left=0.02, bottom=None, right=0.98, top=None,
                    # wspace=0.08, hspace=None)
                    
    #plt.tight_layout()
    plt.savefig(figfname_png, dpi=300)
    plt.savefig(figfname_pdf, dpi=300)
    
    print figfname_png
    #plt.show()
    
    plt.close()
    
    
if __name__=='__main__':
    main(sys.argv)
