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
    
    figfname_png = dir_to_friction_scan_iso + '/friction_maximal_dissipation_overmaterial.png'
    figfname_pdf = dir_to_friction_scan_iso + '/friction_maximal_dissipation_overmaterial.pdf'
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
    
    from latexify import latexify; latexify(scale = 2, fig_height=1.5)
    axes = plt.gca()
    axes.grid(True, linewidth = 0.25, color='grey')
    axes.set_axisbelow(True)
    
    
    for inds, surface in enumerate(dirlist):
        print dir_to_friction_scan_iso, surface, shape_id
        
        avgs = []
        vel_vecs = []
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
            
        # print  v * f - max(v * f*) >= 0
        diffs = []
        for i,deg in enumerate(rangex):
            th = np.deg2rad(deg)
            start_pos = [np.sin(th), np.cos(th)]
            end_pos = [np.sin(th+np.pi), np.cos(th+np.pi)]
            vel_vec = -(np.array(end_pos) - np.array(start_pos))
            
            force = avgs[i]
            #import pdb; pdb.set_trace()
            max_power = -np.inf
            for j,deg_p in enumerate(rangex):
                tmp_power = np.dot(vel_vec, avgs[j])
                if tmp_power > max_power:
                    max_power = tmp_power
                
            diffs.append(np.dot(vel_vec, force) - max_power)
            
        plt.errorbar(rangex, diffs, color='k',  fmt=linestyles[inds]+markerstyles[inds], 
         label=surface, markevery = markeverys[inds], markersize=5, linewidth=0.5)

    plt.tight_layout()
    axes.set_xlabel('sliding direction in angles (deg)')
    axes.set_ylabel('$\Delta P$ (m*N/s)')
        
    legend = plt.legend(loc='lower right', ncol = 4)
    
    legend.get_frame().set_linewidth(0.25)
    plt.subplots_adjust(left=0.12, bottom=0.19, right=None, top=None,
                wspace=None, hspace=None)
    plt.savefig(figfname_png)
    plt.savefig(figfname_pdf)
    plt.show()
    

if __name__=='__main__':
    main(sys.argv)
