#!/usr/bin/env python

# Peter KT Yu, Jan 2017
# rosservice call /wsg_32_driver/move 20 100
import sys, os, json, optparse, copy
from ik.roshelper import ROS_Wait_For_Msg, lookupTransform, lookupTransformList, coordinateFrameTransformList
from ik.ik import setSpeed as setSpeedRos
import rospy, tf
import numpy as np
from numpy import array as npa
import tf.transformations as tfm
import roslib; roslib.load_manifest("robot_comm");
from robot_comm.srv import *
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
from wsg_32_common.srv import Move
import sensor_msgs.msg, geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
import scipy.io as sio
from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker, createPointMarker, createArrowMarker, createSphereMarker
from tf.broadcaster import TransformBroadcaster
import config.helper as helper
from ik.helper import *
from ik.roshelper import *
from config.helper import norm, pause, poselist2mat, mat2poselist

from config.shape_db import ShapeDB
from config.probe_db import ProbeDB
from config.surface_db import SurfaceDB

from apriltags.msg import AprilTagDetections

setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
setZeroL = rospy.ServiceProxy('/ft_left/zero', Zero)
setZeroR = rospy.ServiceProxy('/ft_right/zero', Zero)
setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)
setAccRos = rospy.ServiceProxy('/robot2_SetAcc', robot_SetAcc)
moveGripper = rospy.ServiceProxy('/wsg_32_driver/move', Move)

def setZero():
    setZeroL()
    setZeroR()

# set the parameters
limit = 10000  # number of data points to be collected
ori_down = [0, 1, 0, 0]
threshold = 0.3  # the threshold force for contact, need to be tuned
step_size = 0.004
explore_radius = 0.12
center_world = [0.350, -0.03, 0]
center_world2 = center_world[0:2]
obj_init_frame = []
obj_init_frame2 = []
global_high_vel = 200      # speed for moving up
global_vel = 100           # speed for moving around
global_slow_vel = 60       # speed for exploration
global_super_slow_vel = 5  # speed for reset FT
muc = 0.21  # friction coeff between object and probe
global_frame_id = helper.global_frame_id
data_base = os.environ['DATA_BASE']
offset = center_world[0:2]

hasRobot = True
sleepForFT = 0.1 if hasRobot else 0
sleepForAP = 0.2 if hasRobot else 0

def setSpeed(tcp, ori=30):
    setSpeedRos(tcp=tcp, ori=ori)

def setCart(pos, ori=ori_down):
    param = (np.array(pos) * 1000).tolist() + qwxyz_from_qxyzw(ori)
    print 'setCart', param
    setCartRos(*param)

if not hasRobot:
    def setSpeed(tcp, ori=30):
        pass

    def setCart(pos):
        pass
    
    def setZone(z):
        pass
        
    def lookupTransform(global_frame_id, obj_frame_id, lr):
        return ((0.3,0,0), (0,0,0,1))
    
    def coordinateFrameTransform(slot_pos_obj, obj_frame_id, global_frame_id, lr):
        return [0.3,0,0]
        
    def setZero():
        pass

def transformFt2Global(ftlist):
    # transform ft data to global frame
    (pos_trasform, ori_trasform) = lookupTransform(global_frame_id, '/link_ft', lr)
    rotmat = tfm.quaternion_matrix(ori_trasform)
    ft_global = np.dot(rotmat, ftlist + [1.0])
    return ft_global[0:3].tolist()

def ftmsg2list(ftmsg):
    return [ftmsg.wrench.force.x, ftmsg.wrench.force.y, ftmsg.wrench.force.z]

def vizBlock(pose, mesh, frame_id):
    """block visualization"""
    meshmarker = createMeshMarker(mesh, 
                              offset=tuple(pose[0:3]), rgba=(0.5,0.5,0.5,1),
                              orientation=tuple(pose[3:7]), frame_id=frame_id)
    meshmarker.id = 200
    vizpub.publish(meshmarker)
    #rospy.sleep(0.05)
    
def vizPoint(pos):
    """point visualization"""
    marker = createSphereMarker(offset=pos, color=[0, 0, 1, 0.5], scale=[0.01,0.01,0.01])
    vizpub.publish(marker)
    marker.id = 201
    #rospy.sleep(0.1)

def vizArrow(start, end):
    """prepare arrow visualization"""
    marker = createArrowMarker(points=start+end, color=[1,0,0,1])
    vizpub.publish(marker)
    marker.id = 202
    #rospy.sleep(0.1)

def getFTmsglist():
    return ftmsg2list(rospy.wait_for_message('/netft_data', WrenchStamped, timeout=0.5))

if not hasRobot:
    def getFTmsglist():
        return [1,0,0]
        
def wait_for_ft_calib():
    if hasRobot:
        rospy.wait_for_message('/ft_left/netft_data', WrenchStamped, timeout=3)
        rospy.wait_for_message('/ft_right/netft_data', WrenchStamped, timeout=3)


def getAveragedFT():
    """average 10 times of the FT measurement"""
    tmpft = np.array([0,0,0])
    nsample = 10
    arrayft = []
    for i in xrange(0, nsample):
        FTlist = getFTmsglist()
        tmpft = tmpft + np.array(getFTmsglist())
        arrayft.append(FTlist)
        
    print 'arrayft:\n', arrayft
    print 'std:\n', np.std(np.array(arrayft), axis = 0).tolist()
    print 'mean:\n', tmpft / nsample
    print 'norm(mean):\n', np.linalg.norm(tmpft / nsample)
    return (tmpft / nsample).tolist()
    
    
def getAveragedFT2():
    """average 10 times of the FT measurement"""
    
    if not hasRobot:
        return [1,0,0]
    
    nsample = 10
    queue = []
    def callback(msg):
        if len(queue) < nsample:
            queue.append(ftmsg2list(msg))
    sub = rospy.Subscriber('/netft_data', WrenchStamped, callback)
    
    while len(queue) < nsample:
        rospy.sleep(0.01)
    
    return np.mean(np.array(queue), axis=0).tolist()

colname =  [
  "x of contact position", 
  "y of contact position", 
  "z of contact position", 
  "x of contact normal (in simulator) or force (in real)", 
  "y of contact normal (in simulator) or force (in real)", 
  "z of contact normal (in simulator) or force (in real)", 
  "x of ground truth object pose", 
  "y of ground truth object pose", 
  "z of ground truth object pose", 
  "qw of ground truth object pose", 
  "qx of ground truth object pose", 
  "qy of ground truth object pose", 
  "qz of ground truth object pose", 
  "x of pusher position", 
  "y of pusher position", 
  "z of pusher position", 
  "in contact or not", 
  "x of apriltag object pose", 
  "y of apriltag object pose", 
  "z of apriltag object pose", 
  "qw of apriltag object pose", 
  "qx of apriltag object pose", 
  "qy of apriltag object pose", 
  "qz of apriltag object pose", 
  "yaw of apriltag object pose", 
 ], 

def recover(slot_pos_obj, reset):
    z_recover = 0.011 + z  # the height for recovery 
    z_offset = z_recover
    slot_pos_obj = slot_pos_obj + [0]
    _center_world = copy.deepcopy(center_world)
    if reset:
        # move above the slot
        pos_recover_probe_world = coordinateFrameTransformList(slot_pos_obj, obj_frame_id, global_frame_id, lr)
        pos_recover_probe_world[2] = zup
        setCart(pos_recover_probe_world)
        
        # move down a bit in fast speed
        pos_recover_probe_world = coordinateFrameTransformList(slot_pos_obj, obj_frame_id, global_frame_id, lr)
        pos_recover_probe_world[2] = z_offset + 0.02
        setCart(pos_recover_probe_world)
        
        # move down to the slot    
        # speed down
        setSpeed(tcp=global_slow_vel)
        pos_recover_probe_world[2] = z_offset
        setCart(pos_recover_probe_world)
        #pause()
        
        # move to the world center
        pos_recover_probe_target_world = _center_world
        pos_recover_probe_target_world[2] = z_offset
        setCart(pos_recover_probe_target_world)
        
        # speed up
        setSpeed(tcp=global_high_vel)
    
        # move up
        pos_recover_probe_target_world = _center_world
        pos_recover_probe_target_world[2] = zup + 0.06  # 
        setCart(pos_recover_probe_target_world)
        
def parse_args():
    parser = optparse.OptionParser()
    parser.add_option('-s', action="store", dest='shape_id', 
                      help='The shape id e.g. rect1-rect3', default='rect1')
                      
    parser.add_option('', '--probe', action="store", dest='probe_id', 
                      help='The probe id e.g. probe1-5', default='probe6')
                      
    parser.add_option('', '--surface', action="store", dest='surface_id', 
                      help='The surface id e.g. plywood, abs', default='plywood')
                      
    parser.add_option('', '--nstart', action="store", dest='nstart', type="int", 
                      help='The number of beginning rough probe, should be either one or some even number', default=8)
                      
    parser.add_option('', '--nrep', action="store", dest='nrep', type="int", 
                      help='Number of repitition', default=2)
                      
    parser.add_option('', '--stepsize', action="store", dest='step_size', type="float", 
                      help='Step size', default=0.004)
                      
    # parser.add_option('', '--has_robot', action="store", dest='has_robot', type="bool", 
                      # help='If has robot', default=True)
    return parser.parse_args()

# frame_work_robot_2d = [0.350, -0.03, 0]
contact_pts = []
contact_nms = []
all_contact = []

def turn(a):
    x = tfm.quaternion_multiply(a, ori_down)
    return list(npa(x) / norm(x))

def push(start_pose, end_pose):
    global frame_work_robot_2d, zup, z
    global contact_pts, contact_nms, all_contact
    global step_size
    
    
    start_pos = start_pose[0:2]
    end_pos = end_pose[0:2]
    setSpeed(tcp=global_slow_vel)
    setCart(list(start_pos) + [zup], turn(quat_from_yaw(start_pose[2])))  # because tcp is facing down
    # move down, not supposed to make contact
    setCart(list(start_pos) + [z], turn(quat_from_yaw(start_pose[2])))  
    
    setCart(list(end_pos) + [z], turn(quat_from_yaw(end_pose[2])))  
    
    setCart(list(end_pos) + [zup], turn(quat_from_yaw(end_pose[2])))  
    

def push_center(a, b):
    global obj_init_frame
    print [a,b], obj_init_frame
    newpos_a = transform_back_frame2dpose(a, obj_init_frame)
    newpos_b = transform_back_frame2dpose(b, obj_init_frame)
    push(newpos_a, newpos_b)

def main(argv):
    global lr, vizpub, zup, z, obj_frame_id, probe_radius, step_size
    rospy.init_node('contour_follow', anonymous=True)
    
    # prepare the proxy, lr
    lr = tf.TransformListener()
    br = TransformBroadcaster()
    vizpub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.sleep(0.5)
    t = rospy.Time(0)
    (opt, args) = parse_args()
    
    # load parameters about the surface
    surface_id = opt.surface_id
    surface_db = SurfaceDB()
    surface_thick = surface_db.db[surface_id]['thickness']
    
    # load parameters about the probe
    probe_db = ProbeDB()
    probe_id = opt.probe_id
    probe_length = probe_db.db["probe5"]['length']
    probe_radius = probe_db.db[probe_id]['radius']
    ft_length = probe_db.db["probe5"]['ft_length']
    
    # load parameters about the shape
    shape_db = ShapeDB()
    shape_id = opt.shape_id
    obj_frame_id = shape_db.shape_db[shape_id]['frame_id']
    obj_mesh = shape_db.shape_db[shape_id]['mesh']
    obj_slot = shape_db.shape_db[shape_id]['slot_pos']
    
    step_size = opt.step_size
    
    z = probe_length + ft_length + surface_thick + 0.013
    zup = z + 0.05
    # end of preparation
    
    def run_explore(filename):
        global obj_init_frame
        # 0. check if collected or not
        dir_base = data_base + "/multi_pushes_twofinger/"
        jsonfilename = dir_base+'/%s.json' % filename
        matfilename = dir_base+'/%s.mat' % filename
        bagfilename = '%s.bag' % filename
        topics = ['-a']
        if os.path.isfile(dir_base + bagfilename):
            print "skip", filename
            return
        
        # 1. move to startPos
        setSpeed(tcp=global_vel)
        setZone(0)
        start_pos = [center_world[0] , center_world[1]] + [zup]
        
        #moveGripper(20, 100);  # width (mm), speed (mm/s)
        setCart(start_pos)
        setAccRos(1,1)
        setZero()
        wait_for_ft_calib()
        #pause()
        import Tkinter
        import tkMessageBox
        result = tkMessageBox.showinfo("Continue", "Put the object at the starting place")
        
        rosbag_proc = helper.start_ros_bag(bagfilename, topics, dir_base) 
        rospy.sleep(0.5) 
        
        # 1.1 get object init pos as new center
        box_pose_des_global = lookupTransformList(global_frame_id, obj_frame_id, lr)
        obj_init_frame = pose3d_to_pose2d(box_pose_des_global)
        #obj_init_frame = [0.35, -0.035, 0.0] # hack
        
        pushlen = 0.12
        blocklen = 0.045
        first_push_goal = 0
        if shape_id == "ellip2":
            pushlen += 0.01
        
        if shape_id == "butter":
            pushlen += 0.0
            first_push_goal = 0.04
        #return
        # 2. Push make the pushes fixed wrt the first frame.
        # push: move to the startpose up , move down, push to endpose, move up
        # 2.1 
        push_center([0, -pushlen, np.pi/2], [0, -first_push_goal, np.pi/2])
        # 2.2 
        push_center([0,  pushlen, np.pi/2], [0,  blocklen, np.pi/2])
        # 2.3 
        push_center([pushlen,  0, 0], [0, 0, 0])
        # 2.4 
        push_center([-pushlen,  0, 0], [-blocklen, 0, 0])
        # 2.5
        push_center([-pushlen, -pushlen, np.pi/4], [blocklen, blocklen, np.pi/4])
        # 2.6
        push_center([pushlen, pushlen, np.pi/4], [blocklen, blocklen, np.pi/4])
        # 2.7
        push_center([0, pushlen, np.pi/4], [0, blocklen/2, np.pi/4])
        # 2.8
        push_center([0, -pushlen, np.pi/4], [0, -blocklen, np.pi/4])
        
        helper.terminate_ros_node("/record")
        
    # end of run_explore()
    
    for i in xrange(opt.nrep):
        run_explore('multipush_shape=%s_surface=%s_rep=%04d_step=%.4f_april' % (shape_id, surface_id, i, step_size))
        
    
if __name__=='__main__':
    main(sys.argv)


#rosservice call /robot2_SetSpeed 10 1
#rosservice call /robot2_SetZone "mode: 1"


#all_contact(1:2,:);  % x,y of contact position
#all_contact(4:5,:);  % x,y contact normal
#all_contact(7:9,:);  % box x,y
#all_contact(10:13,:);  % box quaternion
#all_contact(14:16,:);  % pusher position


