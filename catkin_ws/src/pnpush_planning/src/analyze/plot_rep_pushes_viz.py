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
from config.shape_db import ShapeDB
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection

def plot_point_cov(points, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma ellipse based on the mean and covariance of a point
    "cloud" (points, an Nx2 array).

    Parameters
    ----------
        points : An Nx2 array of the data points.
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.

    Returns
    -------
        A matplotlib ellipse artist
    """
    pos = points.mean(axis=0)
    cov = np.cov(points, rowvar=False)
    return plot_cov_ellipse(cov, pos, nstd, ax, **kwargs)

def eigsorted(cov):
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    return vals[order], vecs[:,order]


def plot_cov_ellipse(cov, pos, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma error ellipse based on the specified covariance
    matrix (`cov`). Additional keyword arguments are passed on to the 
    ellipse patch artist.

    Parameters
    ----------
        cov : The 2x2 covariance matrix to base the ellipse on
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.

    Returns
    -------
        A matplotlib ellipse artist
    """
    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    if ax is None:
        ax = plt.gca()

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2 * nstd * np.sqrt(vals)
    ellip = mpatches.Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)

    ax.add_artist(ellip)
    return ellip

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('-s', action="store", dest='shape_id', 
                      help='The shape id e.g. rect1-rect3', default='rect1')
                      
    parser.add_option('', '--surface', action="store", dest='surface_id', 
                      help='The surface id e.g. plywood, abs', default='plywood')
                      
    parser.add_option('', '--nrep', action="store", dest='nrep', type='int',
                      help='Repeat how many times', 
                      default=2000)  
    parser.add_option('', '--reptype', action="store", dest='reptype', type='string',
                      help='Repeat type', 
                      default='normal')  
    
    (opt, args) = parser.parse_args()
    
    if len(args) < 1:  # no bagfile name
        parser.error("Usage: plot_rep_pushes.py [dir_to_rep_push] e.g. /home/mcube/pnpushdata/straight_push_rep")  
        return
    dir_to_rep_push = args[0]
    
    vel = 100 #100
    acc = 0
    i = 0
    s = 0.5 #0.1#0.1#0.5 #0.7
    t = -0.175 #-0.349#-0.349 #-0.698# #0
    
    figfname_png = dir_to_rep_push + '/rep_push_viz_%s.png' % opt.surface_id
    figfname_pdf = dir_to_rep_push + '/rep_push_viz_%s.pdf' % opt.surface_id
    figfname_2_png = dir_to_rep_push + '/rep_push_viz_2_%s.png' % opt.surface_id
    figfname_2_pdf = dir_to_rep_push + '/rep_push_viz_2_%s.pdf' % opt.surface_id
    
    figfname_hist_png = dir_to_rep_push + '/rep_push_viz_hist_%s.png' % opt.surface_id
    figfname_hist_pdf = dir_to_rep_push + '/rep_push_viz_hist_%s.pdf' % opt.surface_id
    
    cachefile = '/tmp/plot_rep_push_%s' % opt.surface_id
    import shelve
    if os.path.exists(cachefile):
        f = shelve.open(cachefile)
        vals = f['vals'];
        #trajs = f['trajs'];
        #fts = f['fts']
        opt = f['opt']
        #trajs_tippose = f['trajs_tippose']
        meantraj = f['meantraj']
        meantraj_tippose = f['meantraj_tippose']
    else:
        vals = [] # delta between start and end
        trajs = []
        trajs_tippose = []
        fts = []
        for rep in xrange(opt.nrep):
            print rep
            h5filename = 'motion_surface=%s_shape=%s_a=%.0f_v=%.0f_i=%.3f_s=%.3f_t=%.3f_rep=%04d.h5' % (opt.surface_id, opt.shape_id, acc*1000, vel, i, s, t, rep)
            
            #filename = 'motion_surface=%s_shape=%s_a=%.0f_v=%.0f_i=%.3f_s=%.3f_t=%.3f_rep=%04d.bag' % (opt.surface_id, opt.shape_id, acc*1000, speeds[cnt_acc], i, s, t, rep)
            filepath = '%s/%s/%s/%s/%s' % (dir_to_rep_push,opt.surface_id,opt.shape_id,opt.reptype,h5filename)
            if not os.path.isfile(filepath):
                print 'not exists', filepath
                continue
            
            f = h5py.File(filepath, "r")
            ft_array = f['ft_wrench'].value
            object_pose = f['object_pose'].value
            tip_pose = f['tip_pose'].value
            f.close()
            
            invT0 = np.linalg.inv(matrix_from_xyzrpy(object_pose[0][1:3].tolist() + [0], [0,0,object_pose[0][3]]))
            
            T = matrix_from_xyzrpy(object_pose[-1][1:3].tolist() + [0], [0,0,object_pose[-1][3]])
            T_T0 = np.dot(invT0, T)
            scale, shear, angles, trans, persp = tfm.decompose_matrix(T_T0)
            vals.append(np.append(trans[0:2] ,np.unwrap([angles[2]])))
            
            # extract traj
            #if rep in xrange(500):
            if True:
                traj = []
                for p in object_pose:
                    T = matrix_from_xyzrpy(p[1:3].tolist() + [0], [0,0,p[3]])
                    T_T0 = np.dot(invT0, T)
                    scale, shear, angles, trans, persp = tfm.decompose_matrix(T_T0)
                    traj.append(np.append([p[0]-object_pose[0][0]], np.append(trans[0:2] , np.unwrap([angles[2]]))) )
                trajs.append(traj)
                
                traj_tippose = []
                for tip_pose_ in tip_pose:
                    traj_tippose.append(np.append([tip_pose_[0]-tip_pose[0][0]], np.dot(invT0, tip_pose_[1:3].tolist()+[0,1])))
                trajs_tippose.append(traj_tippose)
            
    
        def computeMeanTraj(trajs):
            lenn = 1000000
            for traj in trajs:
                lenn = min(lenn, len(traj))
                print len(traj)
            ncol = len(trajs[0][0])
            meantraj = np.zeros((lenn, ncol))
            ntraj = len(trajs)
            for traj in trajs:
                meantraj = meantraj + np.array(traj[0:lenn]) / ntraj
                
            return meantraj
            
        meantraj = computeMeanTraj(trajs)
        meantraj_tippose = computeMeanTraj(trajs_tippose)
        
        
        ll = locals()
        '''
        shv = shelve.open(cachefile, 'n')
        for key, val in ll.iteritems():
            try:
                shv[key] = val
            except:
                pass
        shv.close()
        '''
    (x,y,th)=zip(*(vals))
    
    valscov = np.cov(vals, rowvar=0)
    valsmean = np.mean(vals, axis=0)
    print 'covariance\n', valscov
    print 'mean', valsmean
    print 'mean', valsmean[0:2] * 1000, 'mm', np.rad2deg(valsmean[2]), 'deg'
    eigvs,eigvec = eigsorted(valscov[0:2][:,0:2])
    print 'error_trans:', np.sqrt(eigvs[0] + eigvs[1]) *1000 , 'mm'
    print 'error_percent_trans:', np.sqrt(eigvs[0] + eigvs[1]) / np.sqrt(valsmean[0]**2+ valsmean[1]**2) *100 , '%'
    print 'error_rot:', np.rad2deg(np.sqrt(valscov[2][2])), 'deg'
    print 'error_percent_rot:', np.sqrt(valscov[2][2]) / np.sqrt(valsmean[2]**2) *100 , '%'
    
    #from latexify import latexify; latexify(fig_width=3.39, fig_height=3.39*(sqrt(5)-1.0)/2.0*2,scale = 2)
    #from latexify import latexify; latexify(scale = 2)
    

    #### add the object as polygon
    shape_db = ShapeDB()
    shape = shape_db.shape_db[opt.shape_id]['shape'] # shape of the objects presented as polygon.
    shape_type = shape_db.shape_db[opt.shape_id]['shape_type']
    if shape_type == 'poly':
        shape_polygon_3d = np.hstack((np.array(shape), np.zeros((len(shape), 1)), np.ones((len(shape), 1))))
    elif shape_type == 'ellip':
        shape = shape[0]
    elif shape_type == 'polyapprox':
        shape_polygon_3d = np.hstack((np.array(shape[0]), np.zeros((len(shape[0]), 1)), np.ones((len(shape[0]), 1))))
        
    
    part1 = True
    if part1:
        f1, ((ax1, ax2)) = plt.subplots(1, 2, sharex=True, sharey=True)
        #fig = plt.figure()
        plt.sca(ax1)
        ax = ax1
        (tm, x,y,th) = zip(*meantraj)
        line = plt.plot(x, y, '-k')
        plt.setp(line, linewidth=2)
        
        
        ec, fc = 'black','orangered'
        for ii in np.linspace(0, len(meantraj)-1, 30):
            i = int(ii)
            
            if i == 0:
                alpha , fill = (0.3, True)
            elif i == len(meantraj)-1:
                alpha , fill = (0.6, True)
            else:
                alpha , fill = (0.6, False)
                
            T = matrix_from_xyzrpy([x[i], y[i], 0], [0, 0, th[i]])
            shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
            obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
            
            ax.add_patch(obj)
        #####
        
        ###add the probes as circle
        probe_radius = 0.00475
        for ind, ii in enumerate(np.linspace(0, len(meantraj_tippose)-1, 30)):
            i = int(ii)
            if opt.surface_id == 'abs' and ind < 4:   # hack
                continue
            if i == 0:
                alpha , fill = (0.8, False)
            elif i == len(meantraj_tippose)-1:
                alpha , fill = (0.8, False)
            else:
                alpha , fill = (0.8, False)
            circle = mpatches.Circle(meantraj_tippose[i][1:3], probe_radius, color='black', alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
                
            ax.add_patch(circle)
        
        
        plt.axis('equal') 
        plt.axis('off')
        
        # ##2. plot all traj
        ax = ax2
        plt.sca(ax)
        
        for traj in trajs:
            (tm, x,y,th) = zip(*traj)
            plt.plot(x, y, 'g', alpha=0.5)
            
          # ##plot begin and final mean block
        (tm,x,y,th) = zip(*meantraj)
        for i in [0, -1]:
            alpha , fill = (0.6, False)
            T = matrix_from_xyzrpy([x[i], y[i], 0], [0, 0, th[i]])
            shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
            obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid',  zorder=2)
            ax.add_patch(obj)
        
        line = plt.plot(x, y, '-k')
        plt.setp(line, linewidth=2)
        
        plot_cov_ellipse(valscov[0:2][:,0:2], valsmean[0:2], color='orangered', fill=True, alpha=0.9,  zorder=3)
        #import pdb; pdb.set_trace()
        #plot_cov_ellipse(valscov[0:2][:,0:2], meantraj[-1][1:3], color='orangered', fill=True, alpha=0.9,  zorder=3)
        plt.axis('equal') 
        plt.axis('off')
        
        plt.savefig(figfname_png, dpi=200)
        plt.savefig(figfname_pdf)
    
    
    ## 3. plot final poses
    f2, ((ax3, ax4)) = plt.subplots(1, 2, sharex=False, sharey=False)
    
    ax = ax3
    plt.sca(ax)
    (xd,yd,thd)=zip(*(vals))
    ax.scatter(xd,yd, s=0.2, color='k', alpha=1)
    
    ###   plot begin and final mean block
    ec, fc = 'black','orangered'
    (tm,x,y,th) = zip(*meantraj)
    for i in [0,-1]:
        alpha , fill = (0.6, False)
        T = matrix_from_xyzrpy([x[i], y[i], 0], [0, 0, th[i]])
        shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
        #obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
        #ax.add_patch(obj)
    
    ### plot 2 sigma bound
    
    plot_cov_ellipse(valscov[0:2][:,0:2], valsmean[0:2], color='orangered', fill=True, alpha=0.9,  zorder=0)
    ##plot_cov_ellipse(valscov[0:2][:,0:2], valsmean[0:2], 3, color='orangered', fill=True, alpha=0.5,  zorder=0)
    ##ax.add_patch(obj)
        
    ax.set_ylim([0,1000])
    plt.axis('equal') 
    plt.axis('off')
    ##ax2.set_title('Scatter plot: $\Delta x$ versus $\Delta y$')
        
    plt.tight_layout(pad=0, w_pad=0, h_pad=0)
    plt.subplots_adjust(left=0.08, bottom=0.06, right=0.97, top=1.0,
                wspace=0.01, hspace=0.20)
                
    
    ax = ax4
    plt.sca(ax)
    ##   plot begin and final mean block
    (tm,x,y,th) = zip(*meantraj)
    for i in [0,1]:
        alpha , fill = (0.6, False)
        T = matrix_from_xyzrpy([x[i], y[i], 0], [0, 0, th[i]])
        shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
        obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid',  zorder=2)
        ax.add_patch(obj)
    
    line = plt.plot(x, y, '-k')
    plt.setp(line, linewidth=2)
    
    ## plot simulated data
    (x_sim,y_sim,th_sim) = zip(*get_sim_data())
    line_sim = plt.plot(x_sim, y_sim, '--k')
    plt.setp(line_sim, linewidth=2)
    
    T = matrix_from_xyzrpy([x_sim[-1], y_sim[-1], 0], [0, 0, th_sim[-1]])
    shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
    obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='dashed',  zorder=2)
    ax.add_patch(obj)
    ####
    
    #ax.set_ylim([-0.3,0.05])
    plt.axis('equal') 
    plt.axis('off')
    plt.tight_layout(pad=0, w_pad=0, h_pad=0)
    
    # ax.set_xlim([-0.2,0.2])
    # ax.set_ylim([-0.3,0.05])
    plt.savefig(figfname_2_png, dpi=200)
    plt.savefig(figfname_2_pdf)
    plt.show()
    
    #  5-7 plot histogram
    
    f3, ((ax5, ax6, ax7)) = plt.subplots(1, 3, sharex=False, sharey=False)
    plt.sca(ax5)
    plt.locator_params(axis='x',nbins=4)
    n, bins, patches= ax5.hist(np.array(xd)*1000, 200, normed=1, histtype='stepfilled', 
         facecolor='none', label='x', alpha=1)
    ax5.get_yaxis().set_visible(False)
    ax5.set_xlabel('$\Delta x$ (mm)')
    #ax5.set_title('Histogram of $\Delta x$')
    
    plt.sca(ax6)
    plt.locator_params(axis='x',nbins=4)
    n, bins, patches= ax6.hist(np.array(yd)*1000, 200, normed=1, histtype='stepfilled', 
         facecolor='none', label='y', alpha=1)
    ax6.get_yaxis().set_visible(False)
    ax6.set_xlabel('$\Delta y$ (mm)')
    #ax6.set_title('Histogram of $\Delta y$')
    
    plt.sca(ax7)
    plt.locator_params(axis='x',nbins=4)
    n, bins, patches= ax7.hist(np.rad2deg(thd), 200, normed=1, histtype='stepfilled', 
         facecolor='none', label='theta', alpha=1)
    ax7.get_yaxis().set_visible(False)
    ax7.set_xlabel('$\Delta \\theta$ (degree)')
    #ax7.set_title('Histogram of $\Delta \\theta$')
        
    plt.tight_layout(pad=0, w_pad=0, h_pad=0)
    plt.subplots_adjust(left=0.04, bottom=0.23, right=0.96, top=0.87,
                wspace=0.22, hspace=0.20)
    plt.savefig(figfname_hist_png, dpi=200)
    plt.savefig(figfname_hist_pdf)
    plt.show()

def get_sim_data():
    return [[0.000000, 0.000000, 0.000000],
[0.000000, -0.001895, 0.009369],
[0.000018, -0.003789, 0.018775],
[0.000053, -0.005683, 0.028224],
[0.000107, -0.007575, 0.037723],
[0.000178, -0.009465, 0.047279],
[0.000267, -0.011354, 0.056901],
[0.000375, -0.013240, 0.066595],
[0.000500, -0.015123, 0.076370],
[0.000644, -0.017003, 0.086234],
[0.000807, -0.018879, 0.096194],
[0.000987, -0.020751, 0.106260],
[0.001186, -0.022619, 0.116441],
[0.001404, -0.024482, 0.126744],
[0.001641, -0.026339, 0.137179],
[0.001897, -0.028191, 0.147755],
[0.002171, -0.030036, 0.158482],
[0.002465, -0.031874, 0.169368],
[0.002778, -0.033704, 0.180424],
[0.003110, -0.035526, 0.191658],
[0.003462, -0.037339, 0.203080],
[0.003834, -0.039143, 0.214701],
[0.004225, -0.040937, 0.226530],
[0.004636, -0.042719, 0.238578],
[0.005066, -0.044491, 0.250853],
[0.005517, -0.046249, 0.263367],
[0.005988, -0.047995, 0.276129],
[0.006478, -0.049727, 0.289149],
[0.006989, -0.051443, 0.302438],
[0.007520, -0.053144, 0.316005],
[0.008070, -0.054828, 0.329861],
[0.008641, -0.056494, 0.344016],
[0.009231, -0.058142, 0.358479],
[0.009841, -0.059770, 0.373260],
[0.010470, -0.061376, 0.388369],
[0.011118, -0.062961, 0.403815],
[0.011786, -0.064522, 0.419607],
[0.012471, -0.066059, 0.435753],
[0.013175, -0.067571, 0.452264],
[0.013896, -0.069055, 0.469146],
[0.014634, -0.070512, 0.486407],
[0.015389, -0.071938, 0.504055],
[0.016159, -0.073334, 0.522096],
[0.016944, -0.074698, 0.540536],
[0.017742, -0.076029, 0.559382],
[0.018554, -0.077325, 0.578636],
[0.019377, -0.078585, 0.598305],
[0.020211, -0.079808, 0.618390],
[0.021053, -0.080993, 0.638893],
[0.021904, -0.082138, 0.659817],
[0.022761, -0.083243, 0.681159],
[0.023623, -0.084305, 0.702921],
[0.024487, -0.085326, 0.725098],
[0.025353, -0.086303, 0.747688],
[0.026219, -0.087237, 0.770126],
[0.027084, -0.088129, 0.792188],
[0.027944, -0.088977, 0.813887],
[0.028798, -0.089784, 0.835233],
[0.029643, -0.090548, 0.856234],
[0.030476, -0.091271, 0.876893],
[0.031295, -0.091953, 0.897212],
[0.032100, -0.092595, 0.917189],
[0.032887, -0.093198, 0.936821],
[0.033655, -0.093763, 0.956103],
[0.034403, -0.094291, 0.975027],
[0.035130, -0.094783, 0.993586],
[0.035833, -0.095241, 1.011770],
[0.036513, -0.095667, 1.029570],
[0.037169, -0.096061, 1.046976],
[0.037800, -0.096425, 1.063978],
[0.038405, -0.096761, 1.080567],
[0.038984, -0.097070, 1.096733],
[0.039538, -0.097355, 1.112470],
[0.040066, -0.097615, 1.127768],
[0.040568, -0.097853, 1.142623],
[0.041045, -0.098071, 1.157030]]

if __name__=='__main__':
    import sys
    main(sys.argv)
