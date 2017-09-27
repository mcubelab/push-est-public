#!/usr/bin/env python
import numpy as np
from scipy.optimize import least_squares
import tf.transformations as tfm
import cv2
from numpy import array as npa
import os, json


save_dir = os.environ["DATA_BASE"] + "/camera_calib/"
def quat_to_rod(q):
    # q = [qx, qy, qz, qw]
    rot = tfm.quaternion_matrix(q)[0:3][:, 0:3]
    dst, jacobian = cv2.Rodrigues(rot)
    return dst.T.tolist()[0]


def rod_to_quad(r):
    # q = [qx, qy, qz, qw]
    rotmat , jacobian = cv2.Rodrigues(npa(r))
    rotmat = np.append(rotmat, [[0,0,0]], 0)  
    rotmat = np.append(rotmat, [[0],[0],[0],[1]], 1)  
    q = tfm.quaternion_from_matrix(rotmat)
    return q.tolist()

class Program:
    def __init__(self, point3Ds, point2Ds, x0):
        self.point3Ds = point3Ds
        self.point2Ds = point2Ds
        self.x0 = x0

    def obj_func(self, cam):

        fx, fy = cam[0], cam[1]
        cx, cy = cam[2], cam[3]
        distCoeffs = cam[4], cam[5], cam[6], cam[7], cam[8]
        distCoeffs = (0.0, 0.0, 0.0, 0.0, 0.0)   ## hack no distortion
        tvec = cam[9:12]  # x,y,z
        rvec = cam[12:15]  # rodrigues

        # project
        #point2Ds_p = cv.project(point3Ds, cam)
        cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        point2Ds_p, jacobian = cv2.projectPoints(npa(self.point3Ds, dtype=np.float), rvec, tvec, cameraMatrix, distCoeffs)
        #print point2Ds_p
        point2Ds_pp = [list(p[0]) for p in point2Ds_p]
        diff = npa(point2Ds_pp, dtype=np.float) - npa(self.point2Ds, dtype=np.float)
        diff = diff.flatten(1)
        #import pdb;
        #pdb.set_trace()
        #print diff
        res = np.linalg.norm(diff)
        print res / 27.0
        return diff

    def run(self):
        
        res_1 = least_squares(self.obj_func, self.x0)
        
        print '--original--'
        trans =  self.x0[9:12]
        rod =  self.x0[12:15]
        q = rod_to_quad(rod)
        print 'pose', list(trans) + list(q)
        print 'fx,fy,cx,cy,distCoeff[5]', self.x0[0:9]
        
        
        print '\n--optimized--'
        trans = res_1.x[9:12]
        rod = res_1.x[12:15]
        q = rod_to_quad(rod)
        print 'pose', list(trans) + list(q)
        print 'fx,fy,cx,cy,distCoeff[5]', res_1.x[0:9]
        
    
        
        transform = tfm.concatenate_matrices(tfm.translation_matrix(trans), tfm.quaternion_matrix(q))
        inversed_transform = tfm.inverse_matrix(transform)
        translation = tfm.translation_from_matrix(inversed_transform)
        quaternion = tfm.quaternion_from_matrix(inversed_transform)
        pose =  translation.tolist() + quaternion.tolist()
        print 'webcam_T_robot:', " ".join('%.8e' % x for x in pose)
        print 'K: ', [res_1.x[0], 0.0, res_1.x[2], 0.0, res_1.x[1], res_1.x[3], 0.0, 0.0, 1.0]
        print 'P: ', [res_1.x[0], 0.0, res_1.x[2], 0.0, 0.0, res_1.x[1], res_1.x[3], 0.0, 0.0, 0.0, 1.0, 0.0]
        #print res_1
        return res_1.x
        

if __name__ == '__main__':
    color1 = (0,255,255)
    color2 = (0,0,255)
    color3 = (255,0,255)
    #5.08592196e-01 -5.96469288e-01 6.09164354e-01 9.05378371e-01 3.06042346e-02 -5.82887034e-02 -4.19470873e-01
    x0_ext = [5.08592196e-01, -5.96469288e-01, 6.09164354e-01] + quat_to_rod(
        [9.05378371e-01, 3.06042346e-02, -5.82887034e-02, -4.19470873e-01])
    x0_int = [605.22376, 605.37555, 320.96071, 233.59959, 2.671554e-02, 6.672619e-01, -6.263159e-03, 6.014189e-04,
              -2.923799e+00]
    x0 = x0_int + x0_ext
    
    with open(save_dir + 'data.extracted2d.json') as data_file:
        data = json.load(data_file)
        
    point3d = [d["cross3d"][0:3] for d in data]
    point2d = [d["cross2d"] for d in data]
    #print point3d
    p = Program(point3d, point2d, x0)
    cam = p.run()
    
    # show reprojection
    fx, fy = cam[0], cam[1]
    cx, cy = cam[2], cam[3]
    distCoeffs = cam[4], cam[5], cam[6], cam[7], cam[8]
    tvec = cam[9:12]  # x,y,z
    rvec = cam[12:15]  # rodrigues

    # project
    cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    point2Ds_p, jacobian = cv2.projectPoints(npa(point3d, dtype=np.float), rvec, tvec, cameraMatrix, distCoeffs)
    point2Ds_p_nd, jacobian = cv2.projectPoints(npa(point3d, dtype=np.float), rvec, tvec, cameraMatrix, (0.,0.,0.,0.,0.))

    for i, d in enumerate(data):
        image_viz = cv2.imread(save_dir + d['pic_path'])
        
        
        pt_int = tuple([int(round(p)) for p in d['cross2d']])
        cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color1)
        cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color1)
        
        pt_int = tuple([int(round(p)) for p in point2Ds_p_nd[i][0]])
        cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color3)
        cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color3)
        
        pt_int = tuple([int(round(p)) for p in point2Ds_p[i][0]])
        cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color2)
        cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color2)
        cv2.imshow("image", image_viz)
        
        
        while True:
            # display the image and wait for a keypress
            key = cv2.waitKey(3) & 0xFF
            if key == ord("n"):
                break
                
    #cv2.undistortPoints(npa(point3d, dtype=np.float), point2d, K, dist_coef)
