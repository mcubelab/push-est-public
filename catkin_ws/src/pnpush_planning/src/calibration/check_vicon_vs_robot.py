#!/usr/bin/env python

# this is to find out the transform between the vicon frame and robot frame

import tf.transformations as tfm
import numpy as np
from ik.roshelper import lookupTransform
from ik.roshelper import ROS_Wait_For_Msg
from ik.helper import *
import tf
import rospy
from tf.broadcaster import TransformBroadcaster
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *

from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker
from vicon_bridge.msg import Markers

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from rigid_transform_3D import rigid_transform_3D
import tf.transformations as tfm
from ik.ik import setSpeed

#limits = [0.32, 0.38, -0.2, +0.1, 0.27, 0.27]  #[xmin, xmax, ymin, ymax, zmin, zmax]
limits = [0.32, 0.38, -0.2, +0.1, 0.30+0.02-0.05, 0.30+0.02-0.05]  #[xmin, xmax, ymin, ymax, zmin, zmax]
nseg = [3, 3, 1]
nrotate = 8
ori = [0, 0, 1, 0]
globalacc = 2             # some big number means no limit, in m/s^2

rospy.init_node('vicon_vs_robot')
setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
getJoint = rospy.ServiceProxy('/robot2_GetJoints', robot_GetJoints)
setJoint = rospy.ServiceProxy('/robot2_SetJoints', robot_SetJoints)
setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)
setAcc = rospy.ServiceProxy('/robot2_SetAcc', robot_SetAcc)
listener = tf.TransformListener()

def xyztolist(pos):
    return [pos.x, pos.y, pos.z]

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    #print 'setCart', param
    #pause()
    setCartRos(*param)

def norm(vect):
    vect = np.array(vect)
    return np.sqrt(np.dot(vect, vect))

setZone(0)

xs = []
ys = []
zs = []
xt = []
yt = []
zt = []
setSpeed(100, 60)
setAcc(acc=globalacc, deacc=globalacc)

for x in np.linspace(limits[0],limits[1], nseg[0]):
    for y in np.linspace(limits[2],limits[3], nseg[1]):
        for z in np.linspace(limits[4],limits[5], nseg[2]):
            setCart([x,y,z], ori)
            js = getJoint()
            
            for th in np.linspace(-180,180,nrotate):
                setJoint(js.j1, js.j2, js.j3, js.j4, js.j5, th)
                
                # get the marker pos from vicon
                vmarkers = ROS_Wait_For_Msg('/vicon/markers', Markers).getmsg()
                rospy.sleep(0.2)
                #pause()
                try:
                    vmpos = (np.array(xyztolist(vmarkers.markers[-1].translation)) / 1000.0).tolist()
                except:
                    print 'vicon pos is bad, not using this data'
                    continue
                      
                
                # get the marker pos from robot
                #(vicontrans,rot) = lookupTransform('/viconworld','/vicon_tip', listener)
                (vicontrans,rot) = lookupTransform('/map','/vicon_tip', listener)
                vmpos_robot = list(vicontrans)
                
                # test if the marker is tracked or not, skip
                print 'vicon pos %.4f %.4f %.4f' % tuple(vmpos), 'robot pos %.4f %.4f %.4f' % tuple(vmpos_robot)
                if norm(vmpos) < 1e-9:
                    print 'vicon pos is bad, not using this data'
                    continue
                xs = xs + [vmpos[0]]
                ys = ys + [vmpos[1]]
                zs = zs + [vmpos[2]]
                xt = xt + [vmpos_robot[0]]
                yt = yt + [vmpos_robot[1]]
                zt = zt + [vmpos_robot[2]]
            
print 'data length', len(xs)
plt.scatter(xs, ys, c='r', marker='o')
plt.scatter(xt, yt, c='b', marker='o')

plt.show()

viconpts = np.vstack([xs, ys, zs]).T
robotpts = np.vstack([xt, yt, zt]).T

(R,t,rmse) = rigid_transform_3D(viconpts, robotpts)  # then you'll get vicon frame wrt robot frame

Rh = tfm.identity_matrix()
Rh[np.ix_([0,1,2],[0,1,2])] = R
quat = tfm.quaternion_from_matrix(Rh)

print 'vicon_T_robot:', " ".join('%.8e' % x for x in (t.tolist() + quat.tolist()))
print 'rmse:', rmse

