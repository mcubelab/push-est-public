#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# automate the process of pushing against an object and record the data

import sys
import numpy as np
from ik.roshelper import ROS_Wait_For_Msg
from ik.roshelper import lookupTransform
from ik.roshelper import coordinateFrameTransform
from ik.helper import Timer
from ik.ik import setSpeed
from geometry_msgs.msg import WrenchStamped
import tf
import tf.transformations as tfm
import rospy
import json
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
import sensor_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import os
import scipy.io as sio
from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker
from marker_helper import createPointMarker
from marker_helper import createArrowMarker
from marker_helper import createSphereMarker
from tf.broadcaster import TransformBroadcaster
from math import pi
import copy
import subprocess, os, signal
from config.shape_db import ShapeDB
from config.probe_db import ProbeDB
from config.probe_db import ft_length
from config.surface_db import SurfaceDB
import config.helper as helper
from config.helper import norm, pause



# set the parameters
globalvel = 800           # speed for moving around
globalmaxacc = 100        # big number means no limit, in m/s^2
globalacc = 4             # big number means no limit, in m/s^2
global_slow_vel = 30
ori = [0, 0, 1, 0]
center_world = [0.375, 0, 0]
dist_before_contact = 0.02 
skip_when_exists = True
listener = None
hasRobot = True
obj_frame_id = ''
global_frame_id = helper.global_frame_id
obj_slot = None

if hasRobot:
    setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
    setZeroRos = rospy.ServiceProxy('/zero', Zero)
    setAccRos = rospy.ServiceProxy('/robot2_SetAcc', robot_SetAcc)
    setZoneRos = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    print 'setCart', param
    if hasRobot:
        setCartRos(*param)
    
def setZero():
    if hasRobot:
        setZeroRos()
    
def setAcc(acc, deacc):
    if hasRobot:
        setAccRos(acc, deacc)
    
def setZone(zone):
    if hasRobot:
        setZoneRos(zone)

def wait_for_ft_calib():
    if hasRobot:
        ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()

def recover(slot_pos_obj, reset):
    global z, zup
    #import pdb; pdb.set_trace()
    z_recover = 0.016 + z  # the height for recovery 
    z_ofset = z_recover
    slot_pos_obj = slot_pos_obj + [0]
    _center_world = copy.deepcopy(center_world)
    if reset:
        # move above the slot
        pos_recover_probe_world = coordinateFrameTransform(slot_pos_obj, obj_frame_id, global_frame_id, listener)
        pos_recover_probe_world[2] = zup
        setCart(pos_recover_probe_world, ori)
        
        # speed down
        setSpeed(tcp=global_slow_vel, ori=1000)
        
        # move down to the slot    
        pos_recover_probe_world = coordinateFrameTransform(slot_pos_obj, obj_frame_id, global_frame_id, listener)
        pos_recover_probe_world[2] = z_ofset
        setCart(pos_recover_probe_world, ori)
        #pause()
        
        # move to the world center
        pos_recover_probe_target_world = _center_world
        pos_recover_probe_target_world[2] = z_ofset
        setCart(pos_recover_probe_target_world, ori)
        
        # speed up
        setSpeed(tcp=globalvel, ori=1000)
    
        # move up
        pos_recover_probe_target_world = _center_world
        pos_recover_probe_target_world[2] = zup + 0.06  # 
        setCart(pos_recover_probe_target_world, ori)
    #setCart([center_world[0], center_world[1], z + 0.05], ori)  # move back to let vicon see the marker
    
def polyapprox(shape, s):
    ss = shape[0]
    accu = []
    length = 0
    for i in range(len(ss)):
        accu.append(norm(np.array(ss[(i+1) % len(ss)])-np.array(ss[i])) + length)
        length = accu[-1]
    targetlength = s*length
    ind = 0
    
    for i in range(len(ss)):
        if accu[i] > targetlength:
            ind = i
            break
    
    seglength = norm(np.array(ss[(ind+1) % len(ss)])-np.array(ss[ind]))
    t = (targetlength-accu[ind]) / seglength
    pos = np.array(ss[ind]) * (1-t) +  np.array(ss[(ind+1) % len(ss)]) * t
    pos = np.append(pos, [0])
    tangent = np.array(ss[(ind+1) % len(ss)])-np.array(ss[ind])
    normal = np.array([tangent[1], -tangent[0]]) 
    normal = normal / norm(normal)  # normalize it
    normal = np.append(normal, [0])
    return (pos, normal)

# check whether the probe will hit the object
def polyapprox_check_collision(shape, pos_start_probe_object, probe_radius):
    ss = shape[0]
    # find the closest point on the shape to pos_start_probe_object
    min_dist = 10000
    min_ind = 0
    safety_margin = 0.005
    for i in range(len(ss)):
        dist = norm(np.array(ss[i]) - np.array(pos_start_probe_object[0:2]))
        if dist < min_dist:
            min_dist = dist
            min_ind = i
            
    tangent = np.array(ss[(min_ind+1) % len(ss)])-np.array(ss[min_ind])
    normal = np.array([tangent[1], -tangent[0]]) # pointing out of shape
    normal = normal / norm(normal)  # normalize it
    d = np.dot(normal, np.array(pos_start_probe_object[0:2]) - np.array(ss[min_ind]) )
    if d < probe_radius + safety_margin:
        return True
    else:
        return False

def run_it(accelerations, speeds, shape, nside, side_params, angles, nrep, shape_type, probe_radius, dir_save_bagfile, dist_after_contact, allowed_distance, opt):
    # hack to restart the script to prevent ros network issues.
    global pub
    limit = 100
    cnt = 0
    topics = ['-a']
    
    # enumerate the possible trajectories
    for cnt_acc, acc in enumerate(accelerations):
        # enumerate the side we want to push
        for i in range(nside):
            # enumerate the contact point that we want to push
            for s in side_params:
                if shape_type == 'poly':
                    pos = np.array(shape[i]) *(1-s) + np.array(shape[(i+1) % len(shape)]) *(s)
                    pos = np.append(pos, [0])
                    tangent = np.array(shape[(i+1) % len(shape)]) - np.array(shape[i])
                    normal = np.array([tangent[1], -tangent[0]]) 
                    normal = normal / norm(normal)  # normalize it
                    normal = np.append(normal, [0])
                elif shape_type == 'ellip':
                    (a,b) = shape[0][0], shape[0][1]
                    pos = [shape[0][0] * np.cos(s*2*np.pi), shape[0][1] * np.sin(s*2*np.pi), 0]
                    normal = [np.cos(s*2*np.pi)/a, np.sin(s*2*np.pi)/b, 0]
                    normal = normal / norm(normal)  # normalize it
                elif shape_type == 'polyapprox':
                    pos, normal = polyapprox(shape, s)
                    
                # enumerate the direction in which we want to push
                for t in angles:
                    for rep in xrange(nrep):
                        if nrep > 1:
                            bagfilename = 'motion_surface=%s_shape=%s_a=%.0f_v=%.0f_i=%.3f_s=%.3f_t=%.3f_rep=%04d.bag' % (opt.surface_id, opt.shape_id, acc*1000, speeds[cnt_acc], i, s, t, rep)
                        else:
                            bagfilename = 'motion_surface=%s_shape=%s_a=%.0f_v=%.0f_i=%.3f_s=%.3f_t=%.3f.bag' % (opt.surface_id, opt.shape_id, acc*1000, speeds[cnt_acc], i, s, t)
                            
                        bagfilepath = dir_save_bagfile+bagfilename
                        # if exists then skip it
                        if skip_when_exists and os.path.isfile(bagfilepath):
                            #print bagfilepath, 'exits', 'skip'
                            continue  
                        # find the probe pos in contact in object frame
                        pos_probe_contact_object = pos + normal * probe_radius
                        # find the start point
                        direc = np.dot(tfm.euler_matrix(0,0,t) , normal.tolist() + [1])[0:3] # in the direction of moving out
                        pos_start_probe_object = pos_probe_contact_object + direc * dist_before_contact
                        
                        if shape_type == 'polyapprox' and polyapprox_check_collision(shape, pos_start_probe_object, probe_radius):
                            print bagfilename, 'will be in collision', 'skip'
                            continue
                        
                        # find the end point
                        pos_end_probe_object = pos_probe_contact_object - direc * dist_after_contact
                        
                        # zero force torque sensor
                        rospy.sleep(0.1)
                        setZero()
                        wait_for_ft_calib()
                        
                        # transform start and end to world frame
                        if hasRobot:
                            pos_start_probe_world = coordinateFrameTransform(pos_start_probe_object, obj_frame_id, global_frame_id, listener)
                            pos_end_probe_world = coordinateFrameTransform(pos_end_probe_object, obj_frame_id, global_frame_id, listener)
                            pos_contact_probe_world = coordinateFrameTransform(pos_probe_contact_object, obj_frame_id, global_frame_id, listener)
                            pos_center_obj_world = coordinateFrameTransform([0,0,0], obj_frame_id, global_frame_id, listener)
                        else:
                            pos_start_probe_world = pos_start_probe_object + center_world
                            pos_end_probe_world = pos_end_probe_object + center_world
                            pos_contact_probe_world = pos_probe_contact_object + center_world
                            pos_center_obj_world = center_world

                        # start bag recording
                        # move to startPos
                        
                        start_pos = copy.deepcopy(pos_start_probe_world)
                        start_pos[2] = zup
                        setCart(start_pos,ori)
            
                        start_pos = copy.deepcopy(pos_start_probe_world)
                        start_pos[2] = z
                        setCart(start_pos,ori)
                        
                        
                        
                        setSpeed(tcp=global_slow_vel, ori=1000) # some slow speed
                        mid_pos = copy.deepcopy(pos_contact_probe_world)
                        mid_pos[2] = z
                        pub.publish('move_to_mid_pos')
                        setCart(mid_pos,ori)
                        
                        rosbag_proc = helper.start_ros_bag(bagfilename, topics, dir_save_bagfile) 
                        rospy.sleep(0.5) 
                        
                        end_pos = copy.deepcopy(pos_end_probe_world)
                        end_pos[2] = z
                        if acc == 0:  # constant speed
                            setAcc(acc=globalmaxacc, deacc=globalmaxacc)
                            setSpeed(tcp=speeds[cnt_acc], ori=1000)
                        else:  # there is acceleration
                            setAcc(acc=acc, deacc=globalmaxacc)
                            setSpeed(tcp=1000, ori=1000) # some high speed
                            
                        pub.publish('start_pushing')
                        setCart(end_pos,ori)
                        pub.publish('end_pushing')
                        # end bag recording
                        helper.terminate_ros_node("/record")
                        setSpeed(tcp=globalvel, ori=1000)
                        setAcc(acc=globalacc, deacc=globalacc)
                        
                        # move up vertically
                        end_pos = copy.deepcopy(pos_end_probe_world)
                        end_pos[2] = zup
                        setCart(end_pos,ori)
                        
                        distance_obj_center = np.linalg.norm(np.array(pos_center_obj_world)-np.array(center_world))
                        if abs(t) > 1.3:
                            allowed_distance = 0.00
                        # recover
                        recover(obj_slot, distance_obj_center > allowed_distance)
                        #pause()
                        cnt += 1
                        if cnt > limit:  return
    

import optparse
def main(argv):
    # prepare the proxy, listener
    global globalvel, z, zup, z_ofset, listener, obj_frame_id, obj_slot, pub
    pub = rospy.Publisher('/robot_status', String, queue_size=10)
    
    rospy.init_node('collect_motion_data')
    listener = tf.TransformListener()
    
    parser = optparse.OptionParser()
    parser.add_option('-s', action="store", dest='shape_id', 
                      help='The shape id e.g. rect1-rect3', default='rect1')
                      
    parser.add_option('', '--surface', action="store", dest='surface_id', 
                      help='The surface id e.g. plywood, abs', default='plywood')
                      
    parser.add_option('-r', '--real', action="store_true", dest='real_exp', 
                      help='Do the real experiment space', 
                      default=False)
                      
    parser.add_option('', '--reptype', action="store", dest='reptype', type='string',
                      help='Do the real experiment space', 
                      default='normal')  # normal: no variation, speed, angle, position
                      
    parser.add_option('', '--nrep', action="store", dest='nrep', type='int',
                      help='Repeat how many times', 
                      default=1)  
                      
    parser.add_option('', '--slow', action="store_true", dest='slow', 
                      help='Set slower global speed', 
                      default=False)
                      
    parser.add_option('', '--probe', action="store", dest='probe_id', 
                      help='The probe id e.g. probe1-5', default='probe5')
                      
    (opt, args) = parser.parse_args()
    
    if opt.slow: globalvel = global_slow_vel
    
    probe_db = ProbeDB()
    probe_length = probe_db.db[opt.probe_id]['length']
    
    # parameters about the surface
    surface_id = opt.surface_id
    surface_db = SurfaceDB()
    surface_thick = surface_db.db[surface_id]['thickness']
    
    z = probe_length + ft_length + surface_thick + 0.007   # the height above the table
    z_recover = 0.012 + z  # the height for recovery 
    zup = z + 0.04            # the prepare and end height
    probe_radius = probe_db.db[opt.probe_id]['radius']
    

    # parameters about object
    shape_id = opt.shape_id
    shape_db = ShapeDB()
        
    shape_type = shape_db.shape_db[shape_id]['shape_type']
    shape = shape_db.shape_db[shape_id]['shape']
        
    obj_frame_id = shape_db.shape_db[shape_id]['frame_id']
    obj_slot = shape_db.shape_db[shape_id]['slot_pos']

    # space for the experiment
    real_exp = opt.real_exp
    rep_label = ''
    dist_after_contact = 0.01  #ensure 5mm movement which corresponds 0.25s
    allowed_distance =   0.06  #we want it to change at every time... #0.06
    if real_exp:
        if opt.nrep == 1:
            accelerations = []
            speeds = [20]
            
            if shape_type == 'poly':
                if shape == 'hex':
                    side_params = np.linspace(0, 1, 6)  # decrease the number of pushes
                else:
                    side_params = np.linspace(0, 1, 11)  
            else:
                side_params = np.linspace(0,1,40,endpoint=False)
            
            angles = np.linspace(-1.5, 1.5, 31)
            nside = len(shape)
            #To try
            nside = 1
            #angles = np.linspace(0, 0, 1)
            
        else:
            # set the nominal parameters
            side_params = [0.7]
            angles = [0]
            speeds = [20]
            accelerations = []
            nside = 1
            
            dist_after_contact = 0.15
            #shape = shape[0:1]  # only do it on one side
            
            # what dimension we want to explore
            if opt.reptype == 'normal':
                pass
            elif opt.reptype == 'angle':
                angles = [10, 20, 50]
            elif opt.reptype == 'speed':
                speeds = [10, 20, 50, 75, 100, 150, 200, 300, 400, 500]
            elif opt.reptype == 'position':
                side_params = np.linspace(0.5, 1, 6)
        nspeeds = len(speeds)
        nacc = len(accelerations)
        
        speeds = np.append(speeds, np.repeat(-1, nacc))
        accelerations = np.append(np.repeat(0, nspeeds), accelerations)
    else:
        accelerations = [0, 0, 0.1, 1, 2.5]
        speeds = [50, 400, -1, -1, -1]
        angles = np.linspace(-pi/4, pi/4, 2)
        if shape_type == 'poly':
            side_params = np.linspace(0.1,0.9,1)
        else:
            side_params = np.linspace(0,1,1,endpoint=False)

    # parameters about rosbag
    if opt.nrep == 1:
        dir_save_bagfile = os.environ['PNPUSHDATA_BASE'] + '/validation/%s/%s/' % (surface_id,shape_id)
    else:
        dir_save_bagfile = os.environ['PNPUSHDATA_BASE'] + '/straight_push_rep/%s/%s/%s/' % (surface_id,shape_id,opt.reptype)
    
    setAcc(acc=globalacc, deacc=globalacc)
    setSpeed(tcp=globalvel, ori=1000)
    setZone(0)
    helper.make_sure_path_exists(dir_save_bagfile)
    nrep = 100; # 20 iterations would m
    #using op.nrep = 1 and a new variable nrep
    run_it(accelerations, speeds, shape, nside, side_params, angles, nrep,  
          shape_type, probe_radius, dir_save_bagfile, dist_after_contact, allowed_distance, opt)


if __name__=='__main__':
    main(sys.argv)

