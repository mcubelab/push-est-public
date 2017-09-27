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
                      help='Resolution in deg', nargs=1, default=(5))
    parser.add_option('', '--rep', action="store", type='int', dest='rep', 
                      help='How many repetitions', nargs=1, default=(2))
    parser.add_option('', '--limits', action="store", type='float', dest='limits', 
                      help='Limits in meter', nargs=4, default=(0.250,0.450, -0.233, 0.197)) # [minx, maxx, miny, maxy]
    parser.add_option('', '--N', action="store", type='float', dest='N', 
                      help='Normal force', nargs=1, default=(0.8374 * 9.81)) # Normal force
                      
    (opt, args) = parser.parse_args()
    
    if len(args) < 1:  # no bagfile name
        parser.error("Usage: plot_friction_vs_vel.py [dir_to_friction_scan_iso]")
        return
    
    dir_to_friction_scan_iso = args[0]
    
    figfname_png = dir_to_friction_scan_iso + '/friction_over_direction_overmaterial.png'
    figfname_pdf = dir_to_friction_scan_iso + '/friction_over_direction_overmaterial.pdf'
    ft_wrench = []
    
    min_x, max_x, min_y, max_y = opt.limits
    
    center = [(max_x + min_x) /2, (max_y + min_y) /2 ]
    acc = 0
    
    # a list of materials
    #dirlist = [ name for name in os.listdir(dir_to_friction_scan_iso) if os.path.isdir(os.path.join(dir_to_friction_scan_iso, name)) ]
    
    dirlist = ['abs', 'delrin','plywood',  'pu']
    
    shape_id = 'rect1'
    vel = 20
    N = opt.N
    
    linestyles = [':', '-', '-', '-']
    markerstyles = ['', '', '^', 'o']
    markeverys = [1,1,3,3]
    rangex = xrange(0, 360, opt.res)
    raidus = 0.001
    
    from latexify import latexify; latexify(scale = 2)
    axes = plt.gca()
    axes.grid(True, linewidth = 0.25, color='grey')
    axes.set_axisbelow(True)
    
    for inds, surface in enumerate(dirlist):
        print dir_to_friction_scan_iso, surface, shape_id
        
        avgs = []
        stds = []
        for deg in rangex:
            vals = []
            for rep in xrange(opt.rep):
                h5filename = 'record_surface=%s_shape=%s_a=%.0f_v=%.0f_deg=%d_rep=%03d.h5' % (surface, shape_id, acc*1000, vel, deg, rep)
                filepath = '%s/%s/%s/%s' % (dir_to_friction_scan_iso,surface,shape_id,h5filename)
                f = h5py.File(filepath, "r")
                tip_array = f['tip_array'].value
                ft_wrench = f['ft_wrench'].value
                
                scale = (len(ft_wrench)*1.0/len(tip_array))
                
                for i, tip_pos  in enumerate(tip_array):
                    if np.linalg.norm(np.array(tip_pos[1:3]) - np.array(center)) < raidus:
                        ft_i = int(i * scale)
                        vals.append(ft_wrench[ft_i][1:3])
            avgs.append(np.mean(vals, 0))


            # print surface, ' vel=', v
            # print 'average', '%.3f' % (np.average(vals) / N)
            # print 'std', '%.3f' % (np.std(vals) / N)
            # print 'max', '%.3f' % (np.max(vals) / N)
            # print 'min', '%.3f' % (np.min(vals) / N)
            
            
        
        xs = [ft[0] for ft in avgs]
        ys = [ft[1] for ft in avgs]
        xs.append(xs[0])
        ys.append(ys[0])
    
        plt.errorbar(xs, ys, color='k',  fmt=linestyles[inds]+markerstyles[inds], 
         label=surface, markevery = markeverys[inds], markersize=5, linewidth=0.5)
        
    axes.set_xlim([-7, 7])
    axes.set_ylim([-7, 7])
    axes.set_aspect('equal')
    axes.set_xlabel('Force x')
    axes.set_ylabel('Force y')
        
        
    legend = plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    
    legend.get_frame().set_linewidth(0.25)
    plt.tight_layout(pad=0)
    plt.subplots_adjust(left=0, bottom=None, right=0.87, top=None,
                wspace=None, hspace=None)
    plt.savefig(figfname_png)
    plt.savefig(figfname_pdf)
    plt.show()
    plt.close()
    
    

if __name__=='__main__':
    main(sys.argv)
