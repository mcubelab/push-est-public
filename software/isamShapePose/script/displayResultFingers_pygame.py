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
from ik.helper import wraptopi, matrix_from_xyzrpy
from subprocess import call
import pygame
from pygame.gfxdraw import aapolygon, filled_polygon


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

def show_all_rmse(x_input, x_inc, x_batch, x_true, saveto, show_input, show_batch, label):
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
        
    
    show_rmse(x_true, x_inc, label, saveto)
    if show_batch:
        show_rmse(x_true, x_batch, 'batch', saveto)
    show_rmse(x_true, x_input, 'input', saveto)
    
    if show_input:
        show_error_plot(x_true, x_input, 'input', saveto)
    if show_batch:
        show_error_plot(x_true, x_batch, 'batch', saveto)
    show_error_plot(x_true, x_inc, label, saveto)
    plt.legend()
    plt.savefig(saveto + '.pdf')
    plt.savefig(saveto + '.png', dpi = 300)
    

def plot_pos(pos, ax, color = 'k'):
    l = 0.045
    line = mlines.Line2D([pos[0], pos[0]+l*np.cos(pos[2])], [pos[1], pos[1]+l*np.sin(pos[2])], linestyle = '--', color = color)  # the orientation
    ellip = mpatches.Ellipse(xy=pos[0:2], width=0.002, height=0.002, angle=0, fill = True)  # the position 
    ax.add_artist(line)
    ax.add_artist(ellip)

def make_row_based_npa(a, dim):
    if len(a) == dim:
        return npa(a).T
    return npa(a)

def make_col_based_npa(a, dim):
    import pdb; pdb.set_trace()
    if len(a[0]) == dim:
        return npa(a).T
    return npa(a)

def append_zero_one_horizontally(a):
    return np.hstack((npa(a), np.zeros((len(a), 1)), np.ones((len(a), 1))))

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
GREY = (128, 128, 128)
BLUE =  (28, 57, 187)
GREEN = (192,216,144)
RED =   (255,   0,   0)
LIGHT_YELLOW = (255,255,221)
DARK_RED = (196, 2, 51)
CYAN = (0, 255, 255)

def topixel(a, offset):
    c = npa(a)
    b = npa(a)
    for i in xrange(b.shape[0]):
        b[i,0] = (c[i,0] - offset[0] - (-0.15) ) * (1000.0/(0.15*2))
        b[i,1] = (-(c[i,1] - offset[1]) - (-0.15) ) * (1000.0/(0.15*2))
    return b.astype(int).tolist()
    
def scaletopixel(a):
    return a * (1000.0/(0.15*2))

def drawit_pygame(screen,myfont,i,x_star, x_star_cov, M_star, toplot_star, 
                  x_input, M_input, toplot_input, 
                  x_true, toplot_true, pusher_loc, contact_normal, contact_point, has_contact, has_apriltag,
                  saveto, titletext, saveformat, plotconfig, sub, M_star_3d, M_input_3d, shape, shape_type, shape_polygon_3d, 
                  star_color, index, legend):

    label = mat['label']
    radius = mat['probe_radius']
    startdate = mat['startdate']
    shape_id = mat['shape_id']
    offset = mat['offset']
                      
                      
    textsurface = myfont.render('%d' % i, True, BLACK)
    screen.blit(textsurface,(0,0))

    shift = 0
    shiftx = 750
    filled_polygon(screen, [[shiftx,0+shift],[40+shiftx,0+shift],[40+shiftx,40+shift],[0+shiftx,40+shift]], GREY)
    aapolygon(screen, [[0+shiftx,0+shift],[40+shiftx,0+shift],[40+shiftx,40+shift],[0+shiftx,40+shift]], BLACK)
    textsurface = myfont.render('Groundtruth',  True, BLACK)
    screen.blit(textsurface,(50+shiftx,shift))
    shift = 40    
    filled_polygon(screen, [[0+shiftx,0+shift],[40+shiftx,0+shift],[40+shiftx,40+shift],[0+shiftx,40+shift]], GREEN)
    aapolygon(screen, [[0+shiftx,0+shift],[40+shiftx,0+shift],[40+shiftx,40+shift],[0+shiftx,40+shift]], BLACK)
    textsurface = myfont.render('Input',  True, BLACK)
    screen.blit(textsurface,(50+shiftx,shift))
    shift = 80 + index * 40
    filled_polygon(screen, [[0+shiftx,0+shift],[40+shiftx,0+shift],[40+shiftx,40+shift],[0+shiftx,40+shift]], star_color)
    aapolygon(screen, [[0+shiftx,0+shift],[40+shiftx,0+shift],[40+shiftx,40+shift],[0+shiftx,40+shift]], BLACK)
    textsurface = myfont.render(legend,  True, BLACK)
    screen.blit(textsurface,(50+shiftx,shift))

    # plot groundtruth shape and pose
    
    if toplot_true:
        T = matrix_from_xyzrpy([x_true[i][0], x_true[i][1], 0], [0, 0, x_true[i][2]])
        
        if shape_type == 'poly' or shape_type == 'polyapprox':
            shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
            #pygame.draw.polygon(screen, BLACK, topixel(shape_polygon_3d_world.T[:,0:2], offset).tolist(), 5)
            filled_polygon(screen, topixel(shape_polygon_3d_world.T[:,0:2], offset), GREY)
            aapolygon(screen, topixel(shape_polygon_3d_world.T[:,0:2], offset), BLACK)
            #obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, linewidth=2, linestyle='dashed', fill=False)
        elif shape_type == 'ellip':
            scale, shear, angles, trans, persp = tfm.decompose_matrix(T)
            #obj = mpatches.Ellipse(trans[0:2], shape[0]*2, shape[1]*2, angle=angles[2]/np.pi*180.0, fill=False, linewidth=1, linestyle='solid')
        #ax1.add_patch(obj)
        
    if toplot_star:
        if len(M_star.shape) == 3:
            M_star_i = npa(M_star[i])
            M_star_3d = np.hstack((np.array(M_star_i), np.zeros((len(M_star_i), 1)), np.ones((len(M_star_i), 1))))
        
    # plot input shape and pose
    if toplot_input:
        T = matrix_from_xyzrpy([x_input[i][0], x_input[i][1], 0], [0, 0, x_input[i][2]])
        
        if shape_type == 'poly' or shape_type == 'polyapprox' or shape_type == 'ellip':
            M_input_3d_world = np.dot(T, M_input_3d.T)
            
            filled_polygon(screen, topixel(M_input_3d_world.T[:,0:2], offset), GREEN)
            aapolygon(screen, topixel(M_input_3d_world.T[:,0:2], offset), BLACK)
            #obj = mpatches.Polygon(M_input_3d_world.T[:,0:2], closed=True, linewidth=1, linestyle='solid', fill=False)
            #ax1.plot(M_input_3d_world.T[:,0:1], M_input_3d_world.T[:,1:2], 'go')
        elif shape_type == 'ellip':
            scale, shear, angles, trans, persp = tfm.decompose_matrix(T)
            #obj = mpatches.Ellipse(trans[0:2], shape[0]*2, shape[1]*2, angle=angles[2], fill=False, linewidth=1, linestyle='solid')
        #ax1.add_patch(obj)
       
    # plot estimated shape and pose
    if toplot_star:
        T = matrix_from_xyzrpy([x_star[i][0], x_star[i][1], 0], [0, 0, x_star[i][2]])
        
        if shape_type == 'poly' or shape_type == 'polyapprox' or shape_type == 'ellip':
            M_star_3d_world = np.dot(T, M_star_3d.T)
            #obj = mpatches.Polygon(M_star_3d_world.T[:,0:2], closed=True, linewidth=1, linestyle='solid', fill=False)
            #ax1.plot(M_star_3d_world.T[:,0:1], M_star_3d_world.T[:,1:2], 'ro')
            
            filled_polygon(screen, topixel(M_star_3d_world.T[:,0:2], offset), star_color)
            aapolygon(screen, topixel(M_star_3d_world.T[:,0:2], offset), BLACK)
        elif shape_type == 'ellip':
            scale, shear, angles, trans, persp = tfm.decompose_matrix(T)
            #obj = mpatches.Ellipse(trans[0:2], shape[0]*2, shape[1]*2, angle=angles[2], fill=False, linewidth=1, linestyle='solid')
           
        #ax1.add_patch(obj)
        #return
        # plot the covariance of pose
        #~ if x_star_cov is not None:
            #~ plot_cov_ellipse(npa(x_star_cov[i][0:2][:,0:2]), npa(x_star[i][0:2]), ax = ax1, facecolor = (1,1,153/255.0,0.5))
            #~ plot_cov_fan(x_star_cov[i][2][2], npa(x_star[i][0:3]), npa(x_true[i][0:3]), ax = ax1)
            #~ 
            #~ plot_pos(x_true[i][0:3], ax = ax1)
        #~ 
        #~ plot_pos(x_input[i][0:3], ax = ax1, color = 'green')
            
        
    for side in [0,1]:  # left(0) / right(1)
        # plot probe
        #circle = mpatches.Circle(pusher_loc[side][i], radius = radius)
        #ax1.add_patch(circle)
        center_pix = topixel([pusher_loc[side][i]], offset)
        pygame.gfxdraw.filled_circle(screen, int(center_pix[0][0]), int(center_pix[0][1]), int(scaletopixel(radius)), BLUE)
        pygame.gfxdraw.aacircle(screen, int(center_pix[0][0]), int(center_pix[0][1]), int(scaletopixel(radius)), BLACK)
        #~ 
        if  has_contact[side][i]:
            # plot contact point
            #~ ax1.plot(contact_point[side][i][0], contact_point[side][i][1], 'k*')
            #~ 
            #~ # plot normal
            
            line_pix = topixel([contact_point[side][i], (npa(contact_point[side][i]) + npa(contact_normal[side][i])*0.01).tolist()], offset)
            pygame.gfxdraw.line(screen, line_pix[0][0],line_pix[0][1], line_pix[1][0], line_pix[1][1], GREEN)
            #~ #ax1.arrow(d[i,0], d[i,1], turned_norm[i][0]*0.01, turned_norm[i][1]*0.01, head_width=0.001, head_length=0.01, fc='r', ec='r')
            #~ #ax1.arrow(d[i,0], d[i,1], vicon_norm[i][0]*0.01, vicon_norm[i][1]*0.01, head_width=0.001, head_length=0.01, fc='g', ec='g')
            #~ 
    # plot no apriltag
    if not has_apriltag[i]:
        #ax1.text(offset[0]-0.1, offset[1]-0.1, 'No apriltag')
        textsurface = myfont.render('No apriltag',  True, BLACK)
        screen.blit(textsurface,(0,40))
        
    # no axes
    #ax1.set_axis_off()
        
    #fig1.savefig('%s%07d.png'%(saveto,i), dpi=200, bbox_inches='tight')
    #plt.close(fig1)

def displayResult_pygame(x_star, x_star_cov, x_star_2, x_star_2_cov, M_star, toplot_star, 
                  x_input, M_input, toplot_input, 
                  x_true, pusher_loc, contact_normal, contact_point, has_contact, has_apriltag,
                  saveto, titletext, saveformat, plotconfig, sub):
    # 0. convert from matlab col-based to python row-based
    if toplot_star:
        x_star = make_row_based_npa(x_star, 3)
        x_star_2 = make_row_based_npa(x_star_2, 3)
    
    if toplot_input:
        x_input = make_row_based_npa(x_input, 3)
    
    M_star_3d = None
    if toplot_star:
        if len(M_star.shape) == 2:  # time independent
            M_star = make_row_based_npa(M_star, 2)
            M_star_3d = append_zero_one_horizontally(M_star)
            
        elif len(M_star.shape) == 1:  # no input
            M_star = make_row_based_npa(M_input, 2)
            M_star_3d = append_zero_one_horizontally(M_star)
        
    if toplot_input:
        if len(M_input.shape) == 2: # time independent
            M_input = npa(M_input)
            M_input_3d = append_zero_one_horizontally(M_input)

        
    
    # 2. load shape
    #### add the object as polygon
    shape_db = ShapeDB()
    shape = shape_db.shape_db[shape_id]['shape'] # shape of the objects presented as polygon.
    shape_type = shape_db.shape_db[shape_id]['shape_type']
    #import pdb; pdb.set_trace()
    shape_polygon_3d = None
    if shape_type == 'poly':
        shape_polygon_3d = np.hstack((np.array(shape), np.zeros((len(shape), 1)), np.ones((len(shape), 1))))
    elif shape_type == 'ellip':
        shape = shape[0]
    elif shape_type == 'polyapprox':
        shape_polygon_3d = np.hstack((np.array(shape[0]), np.zeros((len(shape[0]), 1)), np.ones((len(shape[0]), 1))))
    
    # 3. loop through trajectory and plot the shape
    length = len(x_star) if toplot_star else len(x_input)
    pygame.font.init() # you have to call this at the start, 
                       # if you want to use this module.
    myfont = pygame.font.SysFont('Ubuntu', 30)
    pygame.init()
    WHITE = (255, 255, 255)
    size = [1000, 1000]
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption("isam pose estimation display")
    clock = pygame.time.Clock()
    

    #for i in xrange(0,length,sub):
    i = 0
    #
    playmode = False
    pygame.key.set_repeat(2, 2)
    while True:
        screen.fill(LIGHT_YELLOW)
        
        #~ pressed = pygame.key.get_pressed()
        #~ if pressed[pygame.K_LEFT] or pressed[pygame.K_a]:
            #~ i = (i-sub+length) % length
        #~ if pressed[pygame.K_RIGHT]:
            #~ i = (i+sub) % length
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit();
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    i = (i-sub+length) % length
                    playmode = False
                elif event.key == pygame.K_RIGHT:
                    i = (i+sub) % length
                    playmode = False
                elif event.key == pygame.K_p:
                    playmode = True
                elif event.key == pygame.K_SPACE:
                    playmode = False
                elif event.key == pygame.K_0:
                    i = 0
                elif event.key == pygame.K_r:
                    pygame.key.set_repeat(2, 2)
                elif event.key == pygame.K_n:
                    pygame.key.set_repeat()
                    
        if playmode:
            i = (i+1)% length
            
        toplot_true = True  
        drawit_pygame(screen, myfont, i, x_star, x_star_cov, M_star, toplot_star, 
                  x_input, M_input, toplot_input, 
                  x_true, toplot_true, pusher_loc, contact_normal, contact_point, has_contact, has_apriltag,
                  saveto, titletext, saveformat, plotconfig, sub, M_star_3d, M_input_3d, shape, shape_type, shape_polygon_3d,
                  DARK_RED, 0, 'iSAM-estimate')
                  
        # toplot_true = False  # don't repeat
        # toplot_input = False
        # drawit_pygame(screen, myfont, i, x_star_2, x_star_2_cov, M_star, toplot_star, 
                  # x_input, M_input, toplot_input, 
                  # x_true, toplot_true, pusher_loc, contact_normal, contact_point, has_contact, has_apriltag,
                  # saveto, titletext, saveformat, plotconfig, sub, M_star_3d, M_input_3d, shape, shape_type, shape_polygon_3d,
                  # CYAN, 1, 'EKF-estimate')
                  
        pygame.display.flip()
        #clock.tick()
        #print clock.get_fps()
        #print pressed

def show_rmse(x_true, x_star, label, saveto):
    summ = np.array([0., 0., 0.])
    diff_array = []
    for i in xrange(len(x_true)):
        diff = np.array( x_true[i] ) - np.array( x_star[i] ) 
        diff[2] = wraptopi(diff[2])
        summ += np.square(diff)
        diff_array.append([(diff[0]**2 + diff[1]**2)**0.5 * 1000, diff[2]/3.1415*180.0])
    summ /= float(len(x_true))
    sqsum = np.sqrt(summ)
    with open(saveto, 'a+') as f1:
        f1.write("%s RMSE: %g %g (%g mm) %g (%g deg)\n" % 
        (label, sqsum[0], sqsum[1], (sqsum[0]**2 + sqsum[1]**2)**0.5 * 1000, sqsum[2], sqsum[2]/3.1415*180.0))
        f1.write("stds (%g mm) (%g deg)\n" % tuple(np.std(diff_array, axis=0)))
    return summ

def show_error_plot(x_true, x_star, label, saveto):
    xdifflist = []
    ydifflist = []
    thdifflist = []
    for i in xrange(len(x_true)):
        diff = np.array( x_true[i] ) - np.array( x_star[i] ) 
        diff[2] = wraptopi(diff[2])
        xdifflist.append(np.sqrt(diff[0]**2 + diff[1]**2))
        thdifflist.append(np.fabs(diff[2]))
        if np.fabs(diff[2]) > 1.5:
            #import pdb; pdb.set_trace()
            print i, diff[2]
    
    print 'show_error_plot ' + label
    plt.subplot(1, 2, 1)
    plt.plot(range(len(x_true)), xdifflist, label=label, alpha=0.7,linewidth=1.0)
    plt.title('position error')
    plt.xlabel('timestep')
    plt.ylabel('error (m)')
    plt.subplot(1, 2, 2)
    plt.plot(range(len(x_true)), thdifflist, label=label, alpha=0.7,linewidth=1.0)
    plt.title('rotation error ' + label)
    plt.xlabel('timestep')
    plt.ylabel('error (rad)')
        
def mkdir(directory):
    print 'mkdir:', saveto
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
    plt.title('Run time each step (avg=%fms)' % np.std([ss for ss in s if ss > 5]))
    plt.grid(True)
    plt.savefig(saveto)

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

M_star_batch = npa(mat['M_star_batch'])
x_star_batch = npa(mat['x_star_batch']) 
x_star_batch_cov = None #npa(mat['x_star_cov'])
M_star_inc = npa(mat["shape_inc"])
x_star_inc = npa(mat["pose_inc"])
x_star_inc_cov = npa(mat["pose_inc_cov"])
try:
    x_star_ekf = npa(mat["pose_ekf"])
    x_star_ekf_cov = npa(mat["pose_ekf_cov"])
except:
    x_star_ekf = x_star_inc
    x_star_ekf_cov = x_star_inc_cov
    pass
M_input = npa(mat["shape_input"])
x_input = npa(mat["pose_input"])

pusher_loc = mat["pusher"]
contact_normal = mat["contact_normal"]
contact_point = mat["contact_point"]
has_contact = mat["has_contact"]
has_apriltag = mat["has_apriltag"]
x_true = mat["pose_true"]
#d = npa(mat['d'])

label = mat['label']
radius = mat['probe_radius']
startdate = mat['startdate']
shape_id = mat['shape_id']
offset = mat['offset']
inc_time = mat['inc_time']
print 'startdate', startdate
print 'label', label
titletext = '%s-%s/' % (startdate, label)
saveformat = "png" 
plotconfig = ""

saveto =  '%s/result/rmse-%s-%s.txt' % (dbpath, startdate, label)
show_all_rmse(x_input, x_star_inc, x_star_batch, x_true, saveto, show_input=True, show_batch=True, label='inc')

saveto =  '%s/result/rmse-%s-%s-ekf.txt' % (dbpath, startdate, label)
show_all_rmse(x_input, x_star_ekf, x_star_ekf, x_true, saveto, show_input=False, show_batch=False, label='ekf')

saveto =  '%s/result/inc_time_plot-%s-%s.eps' % (dbpath, startdate, label)
show_time_plot(inc_time, saveto)

framerate = "10"
## Show incremental result with input
saveto =  '%s/result/video-%s-%s-inc-withinput/' % (dbpath, startdate, label)
mkdir(saveto)
displayResult_pygame(x_star_inc, x_star_inc_cov, x_star_ekf, x_star_ekf_cov, M_star_inc, True, 
              x_input, M_input, True, 
              x_true, pusher_loc, contact_normal, contact_point, has_contact, has_apriltag,
              saveto, titletext, saveformat, plotconfig, sub)
#ffmpeg_cmd = ["ffmpeg", "-framerate", framerate, "-i", saveto+"%07d.png", "-y", 
#  saveto+"video-%s-%s-inc-withinput.mp4" % (startdate, label)]
#print " ".join(ffmpeg_cmd)
#call(ffmpeg_cmd)
