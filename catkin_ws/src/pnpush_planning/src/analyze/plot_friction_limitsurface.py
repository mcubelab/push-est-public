#!/usr/bin/env python

import optparse, sys
import h5py
import config.helper as helper
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pdb
import glob, os
import ik.helper
import tf.transformations as tfm

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('', '--res', action="store", type='float', dest='res', 
                      help='Resolution in deg', nargs=1, default=(5))
    parser.add_option('', '--rep', action="store", type='int', dest='rep', 
                      help='How many repetitions', nargs=1, default=(1))
    parser.add_option('', '--limits', action="store", type='float', dest='limits', 
                      help='Limits in meter', nargs=4, default=(0.250,0.450, -0.233, 0.197)) # [minx, maxx, miny, maxy]
    parser.add_option('', '--N', action="store", type='float', dest='N', 
                      help='Normal force', nargs=1, default=(0.8374 * 9.81)) # Normal force
    parser.add_option('', '--surface', action="store", type='string', dest='surface_id', 
                      help='surface', nargs=1, default='plywood') # Normal force
                      
    (opt, args) = parser.parse_args()
    
    if len(args) < 1:  # no bagfile name
        parser.error("Usage: plot_friction_limitsurface.py [dir_to_friction_scan_limitsurface]")
        return
    
    dir_to_friction_scan_ls = args[0]
    
    figfname_png = dir_to_friction_scan_ls + '/friction_limitsurface_%s.png' % opt.surface_id
    figfname_pdf = dir_to_friction_scan_ls + '/friction_limitsurface_%s.pdf' % opt.surface_id
    figfname_png_2d = dir_to_friction_scan_ls + '/friction_limitsurface_%s_2d.png' % opt.surface_id
    figfname_pdf_2d = dir_to_friction_scan_ls + '/friction_limitsurface_%s_2d.pdf' % opt.surface_id
    ft_wrench = []
    
    min_x, max_x, min_y, max_y = opt.limits
    
    center = [(max_x + min_x) /2, (max_y + min_y) /2 ]
    acc = 0
    
    # a list of materials
    #dirlist = [ name for name in os.listdir(dir_to_friction_scan_iso) if os.path.isdir(os.path.join(dir_to_friction_scan_iso, name)) ]
    
    #dirlist = ['abs', 'delrin','plywood',  'pu']
    
    shape_id = 'rect1'
    vel = 20
    N = opt.N
    
    linestyles = [':', '-', '-', '-']
    markerstyles = ['', '', '^', 'o']
    markeverys = [1,1,3,3]
    rangex = xrange(0, 360, opt.res)
    thres = 0.001
    degs_default = xrange(0, 360, 5)
    rep = 0
    radii = [0, 0.0125, 0.025, 0.05]
    rotdegs_default = np.linspace(-80, 80, 21)
      
    #hack
    degs_default = [0, 180]
    #radii = [0, 0.05]
    #rotdegs_default = np.linspace(-64, 64, 17)
    
    
    vals = []
    vals_y_extreme = []
    vals_x_extreme = []
    for radius in radii:
        if radius == 0:
            degs = [0]
        else:
            degs = degs_default
        for deg in degs:  # translation velocity direction
            th = np.deg2rad(deg)
            
            if radius == 0:
                rotdegs = [80, -80]
            elif deg in [0, 90, 180, 270]:
                rotdegs = np.linspace(-88, 88, 45)
            else:
                rotdegs = rotdegs_default
                
            for rotdeg in rotdegs:  # rotation velocity direction
                rotth = np.deg2rad(deg)
                start_ori = ik.helper.qwxyz_from_qxyzw(tfm.quaternion_from_matrix((np.dot(tfm.euler_matrix(0,np.pi,0), tfm.euler_matrix(0,0,rotth)))))
                end_ori = ik.helper.qwxyz_from_qxyzw(tfm.quaternion_from_matrix((np.dot(tfm.euler_matrix(0,np.pi,0), tfm.euler_matrix(0,0,-rotth)))))
                start_pos = [np.cos(th)* radius + center[0], np.sin(th)* radius + center[1]]
                end_pos = [np.cos(th+np.pi)* radius + center[0], np.sin(th+np.pi)* radius + center[1]]
                
                vel_direc = [np.array(end_pos) - np.array(start_pos), 2*rotth]
            
                h5filename = 'record_surface=%s_shape=%s_a=%.0f_v=%.0f_deg=%d_rotdeg=%d_radius=%.3f_rep=%03d.h5' % (opt.surface_id, shape_id, acc*1000, vel, deg, rotdeg, radius, rep)
                
                filepath = '%s/%s/%s/%s' % (dir_to_friction_scan_ls,opt.surface_id,shape_id,h5filename)
                print 'processing', filepath
                if not os.path.isfile(filepath):
                    print 'not exists'
                    break
                    
                f = h5py.File(filepath, "r")
                tip_array = f['tip_pose'].value
                ft_wrench = f['ft_wrench'].value
                f.close()
                
                scale = (len(ft_wrench)*1.0/len(tip_array))
                
                for i, tip_pos  in enumerate(tip_array):
                    if radius == 0: # use the center part only
                        if np.linalg.norm(np.array(tip_pos[3]) - np.array(0)) < np.deg2rad(1):
                            ft_i = int(i * scale)
                            vals.append(list(ft_wrench[ft_i][1:3]) + list([ft_wrench[ft_i][3]]))  # force x y and torque in z
                            vals_y_extreme.append(np.abs(ft_wrench[ft_i][3]))
                    else:
                        if np.linalg.norm(np.array(tip_pos[1:3]) - np.array(center)) < thres and np.linalg.norm(np.array(tip_pos[3]) - np.array(0)) < np.deg2rad(1):
                            ft_i = int(i * scale)
                            vals.append(list(ft_wrench[ft_i][1:3]) + list([ft_wrench[ft_i][3]]))  # force x y and torque in z
                            if np.allclose(rotdeg, 0):
                                vals_x_extreme.append(np.linalg.norm(ft_wrench[ft_i][1:3]))
                            #print ft_wrench[ft_i][0] - tip_pos[0]
    
    # from mpl_toolkits.mplot3d import Axes3D
    # 
    # from latexify import latexify; latexify(scale = 2)
    # 
    # fig = plt.figure()
    # axes = fig.add_subplot(111, projection='3d')
    # #axes = plt.gca()
    # axes.grid(True, linewidth = 0.25, color='grey')
    # axes.set_axisbelow(True)
    # 
    # (x,y,z) = zip(*vals)
    # axes.scatter(x, y, z, c=z, marker='.')
    # 
# 
    # #plt.tight_layout()
    # axes.set_xlabel('force x')
    # axes.set_ylabel('force y')
    # axes.set_zlabel('moment')
        # 
    # #legend = plt.legend(loc='lower right', ncol = 4)
    # 
    # #legend.get_frame().set_linewidth(0.25)
    # plt.subplots_adjust(left=0.12, bottom=0.13, right=None, top=None,
                # wspace=None, hspace=None)
    # plt.savefig(figfname_png)
    # plt.savefig(figfname_pdf)
    # plt.show()
    
    
    
    # plot just 2d
    
    
    from latexify import latexify; latexify(scale = 1, fontsize = 14)
    fig = plt.figure()
    axes = plt.gca()
    
    ######
    from matplotlib.patches import Ellipse
    
    w = np.average(vals_x_extreme)*2
    h = np.average(vals_y_extreme)*2
    stdw = np.std(vals_x_extreme)*2
    stdh = np.std(vals_y_extreme)*2
    
    meane = Ellipse(xy=(0,0), width=w, height=h, angle=0, zorder=2)
    ue = Ellipse(xy=(0,0), width=w+stdw*2, height=h+stdh*2, angle=0, zorder=0)
    le = Ellipse(xy=(0,0), width=w-stdw*2, height=h-stdh*2, angle=0, zorder=1)
    
    axes.add_artist(ue)
    ue.set_alpha(0.2)
    ue.set_facecolor((0,0,0.5))
    ue.set_edgecolor('none')
    
    axes.add_artist(le)
    le.set_alpha(1)
    le.set_facecolor((1,1,1))
    le.set_edgecolor('none')
    
    axes.add_artist(meane)
    meane.set_alpha(1)
    meane.set_facecolor('none')
    ##########
    
    
    
    (x,y,z) = zip(*vals)
    plt.scatter(x, z, s=0.01, marker='x', c='k', zorder=3)
    axes.set_xlabel('$f_x$ (N)')
    # if opt.surface_id != 'plywood':
    axes.set_ylabel('$m$ (N $\cdot$ m)')
    # plt.tick_params(
    # axis='x',          # changes apply to the x-axis
    # which='both',      # both major and minor ticks are affected
    # bottom='off',      # ticks along the bottom edge are off
    # top='off'         # ticks along the top edge are off
    # )
    # plt.tick_params(
    # axis='y',          # changes apply to the x-axis
    # which='both',      # both major and minor ticks are affected
    # bottom='off',      # ticks along the bottom edge are off
    # top='off'         # ticks along the top edge are off
    # )
    
    plt.locator_params(axis='both',nbins=4)
    plt.subplots_adjust(left=0.23, bottom=0.24, right=None, top=None,
                wspace=None, hspace=None)
    
    # if opt.surface_id == 'plywood':
        # #axes.get_yaxis().set_visible(False)
        # for xlabel_i in axes.get_yticklabels():
            # xlabel_i.set_fontsize(0.0)
            # xlabel_i.set_visible(False)
            
    if opt.surface_id == 'pu':
        plt.axis([-20, 20, -1.5, 1.5])
    else:
        plt.axis([-5, 5, -0.2, 0.2])
    plt.savefig(figfname_png_2d, dpi = 300)
    plt.savefig(figfname_pdf_2d)
    #plt.show()
    

if __name__=='__main__':
    main(sys.argv)
