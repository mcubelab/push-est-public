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
from ik.helper import *
from config.helper import *

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('-s', action="store", dest='shape_id', 
                      help='The shape id e.g. rect1-rect3', default='rect1')
                      
    parser.add_option('', '--surface', action="store", dest='surface_id', 
                      help='The surface id e.g. plywood, abs', default='plywood')
                      
    parser.add_option('', '--nrep', action="store", dest='nrep', type='int',
                      help='Repeat how many times', 
                      default=5000)  
    parser.add_option('', '--reptype', action="store", dest='reptype', type='string',
                      help='Repeat type', 
                      default='normal')  
    
    (opt, args) = parser.parse_args()
    
    if len(args) < 1:  # no bagfile name
        parser.error("Usage: plot_rep_pushes.py [dir_to_rep_push] e.g. /home/mcube/pnpushdata/straight_push_rep/")  
        return
    dir_to_rep_push = args[0]
    
    vel = 20
    acc = 0
    i = 0
    s = 0.7
    t = 0
    
    figfname_png = dir_to_rep_push + '/rep_push_%s.png' % opt.surface_id
    figfname_pdf = dir_to_rep_push + '/rep_push_%s.pdf' % opt.surface_id

    vals = []
    trajs = []
    fts = []
    for rep in xrange(opt.nrep):
        h5filename = 'motion_surface=%s_shape=%s_a=%.0f_v=%.0f_i=%.3f_s=%.3f_t=%.3f_rep=%04d.h5' % (opt.surface_id, opt.shape_id, acc*1000, vel, i, s, t, rep)
        #filename = 'motion_surface=%s_shape=%s_a=%.0f_v=%.0f_i=%.3f_s=%.3f_t=%.3f_rep=%04d.bag' % (opt.surface_id, opt.shape_id, acc*1000, speeds[cnt_acc], i, s, t, rep)
        filepath = '%s/%s/%s/%s/%s' % (dir_to_rep_push,opt.surface_id,opt.shape_id,opt.reptype,h5filename)
        if not os.path.isfile(filepath):
            print 'not exists', filepath
            break
        
        f = h5py.File(filepath, "r")
        ft_array = f['ft_wrench'].value
        object_pose = f['object_pose'].value
        f.close()
        
        invT0 = np.linalg.inv(matrix_from_xyzrpy(object_pose[0][1:3].tolist() + [0], [0,0,object_pose[0][3]]))
        
        T = matrix_from_xyzrpy(object_pose[-1][1:3].tolist() + [0], [0,0,object_pose[-1][3]])
        T_T0 = np.dot(invT0, T)
        scale, shear, angles, trans, persp = tfm.decompose_matrix(T_T0)
        vals.append(np.append(trans[0:2] * 1000,np.unwrap([angles[2]]) * 180 / np.pi))
        
        # extract traj
        if rep in xrange(1,901,30):
            traj = []
            for p in object_pose:
                T = matrix_from_xyzrpy(p[1:3].tolist() + [0], [0,0,p[3]])
                T_T0 = np.dot(invT0, T)
                scale, shear, angles, trans, persp = tfm.decompose_matrix(T_T0)
                traj.append(np.append([p[0]-object_pose[0][0]], np.append(trans[0:2] * 1000, np.unwrap([angles[2]]) * 180 / np.pi)) )
            trajs.append(traj)
    
            ft_array[:][0] = ft_array[:][0] - ft_array[0][0]
            fts.append(ft_array)
    #import pdb; pdb.set_trace()
    (x,y,th)=zip(*(vals))
    
    print 'covariance\n', np.cov(vals, rowvar=0)
    print 'mean', np.mean(vals, axis = 0)
    
    
    #from latexify import latexify; latexify(fig_width=3.39, fig_height=3.39*(sqrt(5)-1.0)/2.0*2, scale = 2)
    f, ((ax1, ax2, ax3), (ax4, ax5, ax6), (ax7, ax8, ax9), (ax10, ax11, ax12)) = plt.subplots(4, 3)
    n, bins, patches= ax1.hist(x, 200, normed=1, histtype='stepfilled', 
         facecolor='none', label='x', alpha=1)
    ax1.set_title('Histogram of $\Delta x$')
    n, bins, patches= ax2.hist(y, 200, normed=1, histtype='stepfilled', 
         facecolor='none', label='y', alpha=1)
    ax2.set_title('Histogram of $\Delta y$')
    
    n, bins, patches= ax3.hist(th, 200, normed=1, histtype='stepfilled', 
         facecolor='none', label='theta', alpha=1)
    ax3.set_title('Histogram of $\Delta\\theta$')
    
    ax4.scatter(x,y, color='k')
    ax4.set_title('Scatter plot: $\Delta x$ versus $\Delta y$')
    
    ax5.hist2d(x, y, bins=50)
    ax5.set_title('2D Histogram: $\Delta x$ versus $\Delta y$')
    meantraj = np.array(trajs[0])*0
    print len(meantraj)
    #import pdb; pdb.set_trace()
    for i in xrange(16): #(30):
        print i
        lenn = min(len(meantraj), len(trajs[i]))
        meantraj = np.array(meantraj[0:lenn]) + np.array(trajs[i][0:lenn]) / 16.0 #30.0
        
    #import pdb; pdb.set_trace()
        
    lenn = len(meantraj)
    (tm0,x0,y0,th0)=zip(*meantraj)
    for i in xrange(16): #30):
        (tm,x,y,th)=zip(*(trajs[i]))
        ax7.plot(tm[0:lenn], np.array(x[0:lenn])-np.array(x0))
        ax8.plot(tm[0:lenn], np.array(y[0:lenn])-np.array(y0))
        ax9.plot(tm[0:lenn], np.array(th[0:lenn])-np.array(th0))
        
    ax7.set_title('$\Delta x$ over time')
    ax8.set_title('$\Delta y$ over time')
    ax9.set_title('$\Delta \\theta$ over time')
    
    (tm,fx,fy,torq)=zip(*(fts[0]))
    ax10.plot(tm, fx)
    ax11.plot(tm, fy)
    ax12.plot(tm, torq)
    
    plt.tight_layout()
    plt.savefig(figfname_png)
    plt.savefig(figfname_pdf)
    plt.show()
        
if __name__=='__main__':
    import sys
    main(sys.argv)
