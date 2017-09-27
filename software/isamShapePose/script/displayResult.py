#!/usr/bin/python

import scipy.io
import transformations as tfm
import numpy as np
from numpy import array as npa
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import glob, os, sys
from config.shape_db import ShapeDB
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from matplotlib.collections import PatchCollection
from ik.helper import wraptopi

def matrix_from_xyzrpy(translate, rpy):
    return np.dot(tfm.compose_matrix(translate=translate) , 
                   tfm.euler_matrix(*rpy)).tolist()
#offset = [0.35, -0.03]
# x_star: estimated pose [[x,y,theta],..]
# M_star: estimated shape [  ]
# d: measurement data and groundtruth


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

# for plotting variance of rotation
def plot_cov_fan(var, pos, posdrawn, nstd=2, ax=None, **kwargs):
    std = np.sqrt(var)
    angles = np.arange(-std*nstd+pos[2], std*nstd+pos[2], 0.01)
    
    
    # draw each line
    polypt = [[posdrawn[0], posdrawn[1]]]
    l = 0.045
    for a in angles:
        dx = l * np.cos(a)
        dy = l * np.sin(a)
        polypt.append([posdrawn[0] + dx, posdrawn[1] + dy])
    #ax.plot(xs, ys, 'y-')
    
    poly = mpatches.Polygon(xy = npa(polypt), closed=True, edgecolor = 'k', facecolor = (1,1,153/255.0,0.5))  # yellow
    ax.add_artist(poly)

def show_all_rmse(x_input, x_inc, x_batch, d, saveto):
    # 0. convert from matlab col-based to python row-based
    if len(x_inc) == 3:
        x_inc = npa(x_inc).T
    else:
        x_inc = npa(x_inc)
        
    if len(x_batch) == 3:
        x_batch = npa(x_batch).T
    else:
        x_batch = npa(x_batch)

    if len(x_input) == 3:
        x_input = npa(x_input).T
    else:
        x_input = npa(x_input)
        
    # 1. convert groundtruth to list of [x,y,theta]
    true_x = []
    d = d.T
    length = d.shape[0]
    for i in xrange(d.shape[0]):
        matlabq = d[i,10:13].tolist() + [d[i,9].tolist()]
        true_x.append([ d[i][6], d[i][7] , tfm.euler_from_quaternion(matlabq, axes='sxyz')[2] ])
    
    true_x = np.array(true_x)
    
    show_rmse(true_x, x_inc, 'inc', saveto)
    show_rmse(true_x, x_batch, 'batch', saveto)
    show_rmse(true_x, x_input, 'input', saveto)
    
    show_error_plot(true_x, x_inc, 'inc', saveto)
    show_error_plot(true_x, x_batch, 'batch', saveto)
    show_error_plot(true_x, x_input, 'input', saveto)
    plt.legend()
    plt.show()

def plot_pos(pos, ax):
    l = 0.045
    line = mlines.Line2D([pos[0], pos[0]+l*np.cos(pos[2])], [pos[1], pos[1]+l*np.sin(pos[2])], linestyle = '--', color = 'k')  # the orientation
    ellip = mpatches.Ellipse(xy=pos[0:2], width=0.002, height=0.002, angle=0, fill = True)  # the position 
    ax.add_artist(line)
    ax.add_artist(ellip)

def displayResult(x_star, x_star_cov, M_star, toplot_star, x_input, M_input, toplot_input, 
                  d, radius, saveto, titletext, saveformat, plotconfig, shape_id, 
                  offset, sub, turned_norm, vicon_norm, label = ''):
    # 0. convert from matlab col-based to python row-based
    if toplot_star:
        if len(x_star) == 3:
            x_star = npa(x_star).T
        else:
            x_star = npa(x_star)
    
    if toplot_input:
        if len(x_input) == 3:
            x_input = npa(x_input).T
        else:
            x_input = npa(x_input)
        
    if toplot_star:
        if len(M_star.shape) == 2:
            M_star = npa(M_star).T
            M_star_3d = np.hstack((np.array(M_star), np.zeros((len(M_star), 1)), np.ones((len(M_star), 1))))
        
    if toplot_input:
        if len(M_input.shape) == 2:
            M_input = npa(M_input)
            M_input_3d = np.hstack((np.array(M_input), np.zeros((len(M_input), 1)), np.ones((len(M_input), 1))))

    
    # 1. convert groundtruth to list of [x,y,theta]
    true_x = []
    d = d.T
    length = d.shape[0]
    for i in xrange(d.shape[0]):
        #import pdb; pdb.set_trace()
        matlabq = d[i,10:13].tolist() + [d[i,9].tolist()]
        true_x.append([ d[i][6], d[i][7] , tfm.euler_from_quaternion(matlabq, axes='sxyz')[2] ])
    
    true_x = np.array(true_x)
    
    
    # 2. load shape
    #### add the object as polygon
    shape_db = ShapeDB()
    shape = shape_db.shape_db[shape_id]['shape'] # shape of the objects presented as polygon.
    shape_type = shape_db.shape_db[shape_id]['shape_type']
    if shape_type == 'poly':
        shape_polygon_3d = np.hstack((np.array(shape), np.zeros((len(shape), 1)), np.ones((len(shape), 1))))
    elif shape_type == 'ellip':
        shape = shape[0]
    elif shape_type == 'polyapprox':
        shape_polygon_3d = np.hstack((np.array(shape[0]), np.zeros((len(shape[0]), 1)), np.ones((len(shape[0]), 1))))
    
    # 3. loop through trajectory and plot the shape
    length = len(x_star) if toplot_star else len(x_input)
    for i in xrange(0,length,sub):
        if i % 100 == 0:
            print 'display result', i
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, aspect='equal')
        ax1.set_xlim(npa([-0.09,0.09])*1.3 + offset[0])
        ax1.set_ylim(npa([-0.09,0.09])*1.3 + offset[1])
        has_apriltag = d[i][25] > 0.5
        # plot groundtruth shape and pose
        T = matrix_from_xyzrpy([true_x[i][0], true_x[i][1], 0], [0, 0, true_x[i][2]])
        
        if shape_type == 'poly' or shape_type == 'polyapprox':
            shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
            obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, linewidth=2, linestyle='dashed', fill=False)
        elif shape_type == 'ellip':
            scale, shear, angles, trans, persp = tfm.decompose_matrix(T)
            obj = mpatches.Ellipse(trans[0:2], shape[0]*2, shape[1]*2, angle=angles[2]/np.pi*180.0, fill=False, linewidth=1, linestyle='solid')
        ax1.add_patch(obj)
        #fig1.savefig('rect1.png', dpi=200, bbox_inches='tight')
        
        if toplot_star:
            if len(M_star.shape) == 3:
                M_star_i = npa(M_star[i])
                M_star_3d = np.hstack((np.array(M_star_i), np.zeros((len(M_star_i), 1)), np.ones((len(M_star_i), 1))))
            
        # plot input shape and pose
        if toplot_input:
            T = matrix_from_xyzrpy([x_input[i][0], x_input[i][1], 0], [0, 0, x_input[i][2]])
            
            if shape_type == 'poly' or shape_type == 'polyapprox' or shape_type == 'ellip':
                M_input_3d_world = np.dot(T, M_input_3d.T)
                obj = mpatches.Polygon(M_input_3d_world.T[:,0:2], closed=True, linewidth=1, linestyle='solid', fill=False)
                ax1.plot(M_input_3d_world.T[:,0:1], M_input_3d_world.T[:,1:2], 'go')
            elif shape_type == 'ellip':
                scale, shear, angles, trans, persp = tfm.decompose_matrix(T)
                obj = mpatches.Ellipse(trans[0:2], shape[0]*2, shape[1]*2, angle=angles[2], fill=False, linewidth=1, linestyle='solid')
            ax1.add_patch(obj)
           
        # plot estimated shape and pose
        if toplot_star:
            T = matrix_from_xyzrpy([x_star[i][0], x_star[i][1], 0], [0, 0, x_star[i][2]])
            
            if shape_type == 'poly' or shape_type == 'polyapprox' or shape_type == 'ellip':
                M_star_3d_world = np.dot(T, M_star_3d.T)
                obj = mpatches.Polygon(M_star_3d_world.T[:,0:2], closed=True, linewidth=1, linestyle='solid', fill=False)
                ax1.plot(M_star_3d_world.T[:,0:1], M_star_3d_world.T[:,1:2], 'ro')
            elif shape_type == 'ellip':
                scale, shear, angles, trans, persp = tfm.decompose_matrix(T)
                obj = mpatches.Ellipse(trans[0:2], shape[0]*2, shape[1]*2, angle=angles[2], fill=False, linewidth=1, linestyle='solid')
               
            ax1.add_patch(obj)
            
            # plot the covariance of pose
            if x_star_cov is not None:
                plot_cov_ellipse(npa(x_star_cov[i][0:2][:,0:2]), npa(x_star[i][0:2]), ax = ax1, facecolor = (1,1,153/255.0,0.5))
                plot_cov_fan(x_star_cov[i][2][2], npa(x_star[i][0:3]), npa(true_x[i][0:3]), ax = ax1)
                
                plot_pos(true_x[i][0:3], ax = ax1)
                
                
        
        # no axes
        ax1.set_axis_off()
        
        # plot contact point
        ax1.plot(d[i,0], d[i,1], 'k*'); 
        
        # plot probe
        circle = mpatches.Circle((d[i,13:15]), radius = radius)
        ax1.add_patch(circle); 
        if not has_apriltag:
            ax1.text(offset[0]-0.1, offset[1]-0.1, 'No apriltag')
        # plot normal
        #ax1.arrow(d[i,0], d[i,1], d[i,0]+d[i,3]*0.0001, d[i,1]+d[i,4]*0.0001, head_width=0.01, head_length=0.01, fc='k', ec='k')
        ax1.arrow(d[i,0], d[i,1], d[i,3]*0.01, d[i,4]*0.01, head_width=0.001, head_length=0.01, fc='g', ec='g')
        #ax1.arrow(d[i,0], d[i,1], turned_norm[i][0]*0.01, turned_norm[i][1]*0.01, head_width=0.001, head_length=0.01, fc='r', ec='r')
        #ax1.arrow(d[i,0], d[i,1], vicon_norm[i][0]*0.01, vicon_norm[i][1]*0.01, head_width=0.001, head_length=0.01, fc='g', ec='g')
        fig1.savefig('%s%07d.png'%(saveto,i), dpi=200, bbox_inches='tight')
        plt.close(fig1)

def show_rmse(true_x, x_star, label, saveto):
    summ = np.array([0., 0., 0.])
    for i in xrange(len(true_x)):
        diff = np.array( true_x[i] ) - np.array( x_star[i] ) 
        diff[2] = wraptopi(diff[2])
        summ += np.square(diff)
    summ /= float(len(true_x))
    sqsum = np.sqrt(summ)
    with open(saveto, 'a+') as f1:
        f1.write("%s RMSE: %g %g %g\n" % (label, sqsum[0], sqsum[1], sqsum[2]))
    return summ

def show_error_plot(true_x, x_star, label, saveto):
    xdifflist = []
    ydifflist = []
    thdifflist = []
    for i in xrange(len(true_x)):
        diff = np.array( true_x[i] ) - np.array( x_star[i] ) 
        diff[2] = wraptopi(diff[2])
        xdifflist.append(np.sqrt(diff[0]**2 + diff[1]**2))
        thdifflist.append(np.fabs(diff[2]))
    
    print 'show_error_plot ' + label
    plt.subplot(1, 2, 1)
    plt.plot(range(len(true_x)), xdifflist, label=label)
    plt.title('position error')
    plt.xlabel('timestep')
    plt.ylabel('error (m)')
    plt.subplot(1, 2, 2)
    plt.plot(range(len(true_x)), thdifflist, label=label)
    plt.title('rotation error' + label)
    plt.xlabel('timestep')
    plt.ylabel('error (rad)')
        
    
def mkdir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

def show_time_plot(data, saveto):
    import matplotlib.pyplot as plt
    import numpy as np

    t = range(len(data))
    s = np.array(data) * 1000
    plt.plot(t, s)

    plt.xlabel('time step')
    plt.ylabel('time (ms)')
    plt.title('Run time each step (avg=%fms)' % np.average(s))
    plt.grid(True)
    plt.savefig(saveto)
    #plt.show()


dbpath = os.environ['DATA_BASE']
if len(sys.argv) >= 2:
    path = sys.argv[1]
    if not os.path.exists(path):
        path = dbpath + '/result/' + path
else:
    path = dbpath+'/result/shapeRecon-20170106_235716_153-isam.json'
    
    
if len(sys.argv) >= 3:
    sub = int(sys.argv[2])
else:
    sub = 1
    
import json
with open(path) as data_file:    
    mat = json.load(data_file)

x_star = npa(mat['x_star'])
x_star_cov = None #npa(mat['x_star_cov'])
M_star = npa(mat['M_star'])
M_star_inc = npa(mat["shape_inc"])
x_star_inc = npa(mat["pose_inc"])
x_star_inc_cov = npa(mat["pose_inc_cov"])
M_input = npa(mat["shape_input"])
x_input = npa(mat["pose_input"])
d = npa(mat['d'])
label = mat['label']
radius = mat['probe_radius']
startdate = mat['startdate']
shape_id = mat['shape_id']
offset = mat['offset']
turned_norm = mat['turned_norm']
vicon_norm = mat['vicon_norm']
inc_time = mat['inc_time']
print 'startdate', startdate
print 'label', label
titletext = '%s-%s/' % (startdate, label)
saveformat = "png" #mat['saveformat'][0]
plotconfig = ""


from subprocess import call
#sub = 1

saveto =  '%s/result/rmse-%s-%s.txt' % (dbpath, startdate, label)
show_all_rmse(x_input, x_star_inc, x_star, d, saveto)

saveto =  '%s/result/inc_time_plot-%s-%s.eps' % (dbpath, startdate, label)
show_time_plot(inc_time, saveto)

framerate = "10"
## Show incremental result with input
saveto =  '%s/result/video-%s-%s-inc-withinput/' % (dbpath, startdate, label)
mkdir(saveto)
print saveto
displayResult(x_star_inc, x_star_inc_cov, M_star_inc, True, x_input, M_input, True, d, radius, saveto, titletext, saveformat, plotconfig, shape_id, offset, sub, turned_norm, vicon_norm, 'inc')
call(["ffmpeg", "-framerate", framerate, "-i", saveto+"%07d.png", "-y", saveto+"video-%s-%s-inc-withinput.mp4" % (startdate, label)])


## Show batch result
saveto =  '%s/result/video-%s-%s-batch-withinput/' % (dbpath, startdate, label)
mkdir(saveto)
print saveto
displayResult(x_star, x_star_cov, M_star, True, x_input, M_input, True, d, radius, saveto, titletext, saveformat, plotconfig, shape_id, offset, sub, turned_norm, vicon_norm, 'batch')
call(["ffmpeg", "-framerate", framerate, "-i", saveto+"%07d.png", "-y", saveto+("video-%s-%s-batch-withinput.mp4" % (startdate, label))])


## Show incremental result
saveto =  '%s/result/video-%s-%s-inc/' % (dbpath, startdate, label)
mkdir(saveto)
print saveto
displayResult(x_star_inc, x_star_inc_cov, M_star_inc, True, x_input, M_input, False, d, radius, saveto, titletext, saveformat, plotconfig, shape_id, offset, sub, turned_norm, vicon_norm)
call(["ffmpeg", "-framerate", framerate, "-i", saveto+"%07d.png", "-y", saveto+"video-%s-%s-inc.mp4" % (startdate, label)])

## Show batch result
saveto =  '%s/result/video-%s-%s-batch/' % (dbpath, startdate, label)
mkdir(saveto)
print saveto
displayResult(x_star, x_star_cov, M_star, True, x_input, M_input, False, d, radius, saveto, titletext, saveformat, plotconfig, shape_id, offset, sub, turned_norm, vicon_norm)
call(["ffmpeg", "-framerate", framerate, "-i", saveto+"%07d.png", "-y", saveto+("video-%s-%s-batch.mp4" % (startdate, label))])

## Show input
saveto =  '%s/result/video-%s-%s-input/' % (dbpath, startdate, label)
mkdir(saveto)
print saveto
displayResult(x_star, x_star_cov, M_star, False, x_input, M_input, True, d, radius, saveto, titletext, saveformat, plotconfig, shape_id, offset, sub, turned_norm, vicon_norm, 'input')
call(["ffmpeg", "-framerate", framerate, "-i", saveto+"%07d.png", "-y", saveto+("video-%s-%s-input.mp4" % (startdate, label))])
