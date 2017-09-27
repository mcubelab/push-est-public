#!/usr/bin/env python

import optparse, sys
import h5py
import config.helper as helper
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pdb
import glob, os

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('', '--avgcolorbar', action="store", type='float', dest='avgcolorbar', 
                      help='Color bar', nargs=2, default=(None,None))
    parser.add_option('', '--res', action="store", type='float', dest='res', 
                      help='Resolution in meter', nargs=2, default=(0.005,0.005))
    parser.add_option('', '--limits', action="store", type='float', dest='limits', 
                      help='Limits in meter', nargs=4, default=(0.250,0.450, -0.233, 0.197)) # [minx, maxx, miny, maxy]
    parser.add_option('', '--N', action="store", type='float', dest='N', 
                      help='Normal force', nargs=1, default=(0.8374 * 9.81)) # Normal force
                      
    (opt, args) = parser.parse_args()
    
    if len(args) < 1:  # no bagfile name
        parser.error("Usage: plot_friction_vs_vel.py [dir_to_friction_scan_vels]")
        return
    
    friction_scan_vel_dir = args[0]
    
    figfname_png = friction_scan_vel_dir + '/friction_over_speed_overmaterial.png'
    figfname_pdf = friction_scan_vel_dir + '/friction_over_speed_overmaterial.pdf'
    ft_wrench = []
    
    min_x, max_x, min_y, max_y = opt.limits
    min_y += opt.res[1]*40   
    max_y -= opt.res[1]*40  
    
    # a list of materials
    #dirlist = [ name for name in os.listdir(friction_scan_vel_dir) if os.path.isdir(os.path.join(friction_scan_vel_dir, name)) ]
    
    dirlist = ['abs', 'delrin', 'plywood', 'pu']
    
    object_id = 'rect1'
    N = opt.N
    
    from latexify import latexify; latexify()
    linestyles = [':', '-', '-', '-']
    markerstyles = ['', '', '^', 'o']
    for inds, surface in enumerate(dirlist):
        print friction_scan_vel_dir, surface, object_id
        filelist = glob.glob("%s/%s/%s/*.h5" % (friction_scan_vel_dir, surface, object_id))
        rangex = []
        avgs = []
        stds = []
        for filepath in filelist:  #record_surface=delrin_shape=rect1_a=0_v=10_rep=000.h5
            v = int(os.path.basename(filepath).split('_')[4].split('=')[1])
            rangex.append(v)
            vals = []
            
            f = h5py.File(filepath, "r")
            tip_array = f['tip_array'].value
            ft_wrench = f['ft_wrench'].value
            scale = (len(ft_wrench)*1.0/len(tip_array))
            
            
            for i, tip_pos  in enumerate(tip_array):
                #print tip_pos
                #raw_input()
                if tip_pos[1] >= min_x and tip_pos[1] <= max_x and tip_pos[2] >= min_y and tip_pos[2] <= max_y:
                    ft_i = int(i * scale)
                    vals.append(np.fabs(ft_wrench[ft_i][2]))
            avgs.append(np.average(vals) / N)
            stds.append(np.std(vals) / N)

            print surface, ' vel=', v
            print 'average', '%.3f' % (np.average(vals) / N)
            print 'std', '%.3f' % (np.std(vals) / N)
            print 'max', '%.3f' % (np.max(vals) / N)
            print 'min', '%.3f' % (np.min(vals) / N)
            
        
        sorted_avgs = [y for (x,y) in sorted(zip(rangex,avgs))]
        sorted_stds = [y for (x,y) in sorted(zip(rangex,stds))]
        sorted_rangex = sorted(rangex)
        
        print sorted_avgs
        print sorted_stds
        print sorted_rangex
        
        #plt.errorbar(sorted_rangex, sorted_avgs, yerr=sorted_stds, color='k', fmt=linestyles[inds]+'o', label=surface)
        plt.errorbar(sorted_rangex, sorted_avgs, color='k', fmt=linestyles[inds]+markerstyles[inds], 
                     label=surface, markersize=3, linewidth=0.25)
        
    axes = plt.gca()
    axes.set_xlim([0, sorted_rangex[-1]+50])
    axes.set_ylim([0, 2])
    axes.set_xlabel('Speed (mm/s)')
    axes.set_ylabel('Coefficient of friction')
        
    legend = plt.legend(ncol=2, loc='upper left')
    legend.get_frame().set_linewidth(0.25)
    plt.tight_layout(pad=0.1)
    plt.savefig(figfname_png)
    plt.savefig(figfname_pdf)
    plt.close()
    
    

if __name__=='__main__':
    main(sys.argv)
