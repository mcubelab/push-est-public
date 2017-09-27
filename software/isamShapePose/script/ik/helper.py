import os, json, sys, time
import numpy as np
import tf.transformations as tfm
from numpy import linalg as la
import traceback

listener = None

def xyth_from_xyzquat(pose):
    tmp = xyzrpy_from_xyzquat(pose)
    return tmp[0:2] + [tmp[5]]
    
def xyth_from_xyzqwxyz(pose):
    tmp = xyzrpy_from_xyzqwxyz(pose)
    return tmp[0:2] + [tmp[5]]

def xyzrpy_from_xyzquat(pose):
    return list(pose[0:3]) + list(tfm.euler_from_quaternion(pose[3:7])) # x,y,z,qx,qy,qz,qw
    
def xyzrpy_from_xyzqwxyz(pose):
    pose = list(pose)
    return pose[0:3] + list(tfm.euler_from_quaternion(pose[4:7] + [pose[3]])) # x,y,z,qx,qy,qz,qw

def qwxyz_from_qxyzw(pose):
    return [pose[3]]+list(pose[0:3])
    
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
## f: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    pts_array = np.array(pts).reshape(-1, 2)
    theta = f[2]
    c = np.cos(theta)
    s = np.sin(theta)
    T = np.array([[c, -s, f[0]],
                  [s,  c, f[1]],
                  [0,  0,   1]])
    pts_ret = np.linalg.solve(T, np.vstack((pts_array.T, np.ones((1, pts_array.shape[0])))))
    return __reduce(pts_ret[0:2,:].T).tolist()

def transform_to_frame2d_matrix(f):
#transform_to_frame2d (pts, f)
## pts: row vectors [x,y]
## f: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    pts_array = np.array(pts).reshape(-1, 2)
    theta = f[2]
    c = np.cos(theta)
    s = np.sin(theta)
    T = np.array([[c, -s, f[0]],
                  [s,  c, f[1]],
                  [0,  0,   1]])
    return np.linalg.inv(T)


def transform_pose_to_frame2d(pose, f):
#transform_to_frame2d (pts, f)
## pose: list [x,y,theta]
## f: list vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    p = [pose[0] - f[0], pose[1] - f[1]]
    theta = f[2]
    c, s = np.cos(theta), np.sin(theta)
    invT = np.array([[c,   s],
                     [-s,  c]])
    p = np.dot(invT, p)
    return [p[0], p[1], pose[2] - f[2]]

def matrix2d_to_pose(m):
    return [m[0,2], m[1,2], np.atan2(m[1,0], m[0,0])]
    
def transform_back_frame2d(pts, f):
#transform_back_frame2d (pts, f)
## pts: row vectors [x,y]
## f: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    pts_array = np.array(pts).reshape(-1, 2)
    theta = f[2]
    c = np.cos(theta)
    s = np.sin(theta)
    T = np.array([[c, -s, f[0]],
                  [s,  c, f[1]],
                  [0,  0,   1]])
    pts_ret = np.dot(T, np.vstack((pts_array.T, np.ones((1, pts_array.shape[0])))))
    return __reduce(pts_ret[0:2,:].T).tolist()


def rotate_to_frame2d(pts, f):
#rotate_to_frame2d (pts, f)
## pts: row vectors [x,y]
## x: row vector [x,y,theta] which defines a 2d frame
## return: row point vectors in frame f, [x,y]
    pts_array = np.array(pts).reshape(-1, 2)
    theta = f[2]
    c = np.cos(theta)
    s = np.sin(theta)
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
    c = np.cos(theta)
    s = np.sin(theta)
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

def wraptopi(data):
    return ( data + np.pi) % (2 * np.pi ) - np.pi 
