import os
import json
import sys
import pdb
import numpy as np
import tf
import rospy
import tf.transformations as tfm
import time
from numpy import linalg as la
import traceback
from roshelper import lookupTransform
from math import *

listener = None

def xyzrpy_from_xyzquat(pose):
    return pose[0:3] + list(tfm.euler_from_quaternion(pose[3:7])) # x,y,z,qx,qy,qz,qw

def quat_from_yaw(yaw):
    return list(tfm.quaternion_from_euler(0,0,yaw)) # x,y,z,qx,qy,qz,qw

def yaw_from_quat(quat):
    return list(tfm.euler_from_quaternion(quat))[2] # x,y,z,qx,qy,qz,qw
    
def pose3d_to_pose2d(pose):
    return pose[0:2] + [yaw_from_quat(pose[3:7])]

def qwxyz_from_qxyzw(pose):
    return [pose[3]]+list(pose[0:3])
    
def qxyzw_from_qwxyz(pose):
    return list(pose[1:4]) + [pose[0]]
    
def matrix_from_xyzquat(translate, quaternion):
    return np.dot(tfm.compose_matrix(translate=translate) , 
                   tfm.quaternion_matrix(quaternion)).tolist()
                   
def matrix_from_xyzrpy(translate, rpy):
    return np.dot(tfm.compose_matrix(translate=translate) , 
                   tfm.euler_matrix(*rpy)).tolist()
                   
def rotmatrix_from_quat(quaternion):
    return (tfm.quaternion_matrix(quaternion)).tolist()

def transformBack(tf_xyzquat, pose):
    T_mat = tfm.concatenate_matrices( tfm.translation_matrix(tf_xyzquat[0:3]), tfm.quaternion_matrix(tf_xyzquat[3:7]))
    pose_mat = tfm.concatenate_matrices( tfm.translation_matrix(pose[0:3]),  tfm.quaternion_matrix(pose[3:7]) )
    new_pose_mat = np.dot(pose_mat, tfm.inverse_matrix(T_mat))
    return tfm.translation_from_matrix(new_pose_mat).tolist() + tfm.quaternion_from_matrix(new_pose_mat).tolist()


def transform_back(pose1, frame):
    pose1_mat = tfm.concatenate_matrices( tfm.translation_matrix(pose1[0:3]), tfm.quaternion_matrix(pose1[3:7]))
    frame_mat = tfm.concatenate_matrices( tfm.translation_matrix(frame[0:3]),  tfm.quaternion_matrix(frame[3:7]) )
    new_pose_mat = np.dot(frame_mat, pose1_mat)
    return tfm.translation_from_matrix(new_pose_mat).tolist() + tfm.quaternion_from_matrix(new_pose_mat).tolist()

# something useful for building primitives
import geometry_msgs.msg

def pause():
    raw_input('Press any key to continue')

    
def getObjCOM(objPose, objId):
    #gives you the center of mass of the object    
    # object frame is attached at com
    objPosition = objPose[0:3]
    return objPosition

# if it is 1*n then return 1-d
def __reduce(nparray):
    if nparray.shape[0] == 1:
        return nparray.reshape(-1)
        
def transform_to_frame2d(pts, f):
#transform_to_frame2d (pts, f)
## pts: row vectors [x,y]
## x: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    #if np.array(pts).ndim == 1: pts = [pts]
    pts_array = np.array(pts).reshape(-1, 2)
    theta = f[2]
    c = cos(theta)
    s = sin(theta)
    T = np.array([[c, -s, f[0]],
                  [s,  c, f[1]],
                  [0,  0,   1]])
    pts_ret = np.linalg.solve(T, np.vstack((pts_array.T, np.ones((1, pts_array.shape[0])))))
    return __reduce(pts_ret[0:2,:].T).tolist()

def transform_to_frame2dpose(pose, f):
#transform_to_frame2d (pts, f)
## pose: row vectors [x,y,theta]
## x: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y,theta]
    theta = f[2]
    c = cos(theta)
    s = sin(theta)
    px = pose[0] - f[0]
    py = pose[1] - f[1]
    return [px*c + py*s, -px*s + py*c, pose[2]-theta]

def transform_back_frame2d(pts, f):
#transform_back_frame2d (pts, f)
## pts: row vectors [x,y]
## x: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    pts_array = np.array(pts).reshape(-1, 2)
    theta = f[2]
    c = cos(theta)
    s = sin(theta)
    T = np.array([[c, -s, f[0]],
                  [s,  c, f[1]],
                  [0,  0,   1]])
    pts_ret = np.dot(T, np.vstack((pts_array.T, np.ones((1, pts_array.shape[0])))))
    return __reduce(pts_ret[0:2,:].T).tolist()


def transform_back_frame2dpose(pose, f):
#transform_back_frame2dpose (pose, f)
## pose: row vector [x,y, theta]
## x: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    theta = f[2]
    c = cos(theta)
    s = sin(theta)
    return [c*pose[0] - s*pose[1] + f[0], s*pose[0] + c*pose[1] + f[1], pose[2] + theta]

def rotate_to_frame2d(pts, f):
#rotate_to_frame2d (pts, f)
## pts: row vectors [x,y]
## x: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    pts_array = np.array(pts).reshape(-1, 2)
    theta = f[2]
    c = cos(theta)
    s = sin(theta)
    T = np.array([[c,   s,   0],
                  [-s,  c,   0],
                  [0,  0,   1]])
    pts_ret = np.dot(T, np.vstack((pts_array.T, np.ones((1, pts_array.shape[0])))))
    return __reduce(pts_ret[0:2,:].T).tolist()


def rotate_back_frame2d(pts, f):
#rotate_back_frame2d (pts, f)
## pts: row vectors [x,y]
## x: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    pts_array = np.array(pts).reshape(-1, 2)
    theta = f[2]
    c = cos(theta)
    s = sin(theta)
    T = np.array([[c, -s,   0],
                  [s,  c,   0],
                  [0,  0,   1]])
    pts_ret = np.dot(T, np.vstack((pts_array.T, np.ones((1, pts_array.shape[0])))))
    return __reduce(pts_ret[0:2,:].T).tolist()
    
    
def poselist2mat(pose):
    return np.dot(tfm.translation_matrix(pose[0:3]), tfm.quaternion_matrix(pose[3:7]))

def mat2poselist(mat):
    pos = tfm.translation_from_matrix(mat)
    quat = tfm.quaternion_from_matrix(mat)
    return pos.tolist() + quat.tolist()

def norm(vect):
    vect = np.array(vect)
    return np.sqrt(np.dot(vect, vect))

def quat_from_matrix(rot_matrix):
    return (tfm.quaternion_from_matrix(rot_matrix))


def rotmatY(theta):
    theta_rad=(theta*np.pi)/180
    return(np.array([[np.cos(theta_rad), 0, np.sin(theta_rad)],
    [0, 1, 0],[-np.sin(theta_rad), 0, np.cos(theta_rad)]]))


def rotmatX(theta):
    theta_rad=(theta*np.pi)/180
    return(np.array([[1, 0, 0],
    [0, np.cos(theta_rad), -np.sin(theta_rad)],[0, np.sin(theta_rad), np.cos(theta_rad)]]))

def rotmatZ(theta):
    theta_rad=(theta*np.pi)/180
    return(np.array([[np.cos(theta_rad), -np.sin(theta_rad), 0],[np.sin(theta_rad), np.cos(theta_rad), 0], [0,0,1]]))
    
class Timer(object):
    #### Timer to time a piece of code
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print '\t[%s]' % self.name,
        print 'Elapsed: %s' % (time.time() - self.tstart)

class shortfloat(float):
    def __repr__(self):
        return "%0.3f" % self
