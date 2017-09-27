#!/usr/bin/env python

# this is to find out the transform between the vicon frame and robot frame

import tf.transformations as tfm
import numpy as np
from ik.roshelper import lookupTransform
from ik.roshelper import ROS_Wait_For_Msg
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

limits = [0.2, 0.4, -0.3, +0.3, 0.218, 0.218]  #[xmin, xmax, ymin, ymax, zmin, zmax]
ori = [0, 0.7071, 0.7071, 0]

rospy.init_node('vicon_vs_robot')
setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)
listener = tf.TransformListener()

def xyztolist(pos):
    return [pos.x, pos.y, pos.z]

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    #print 'setCart', param
    #pause()
    setCartRos(*param)



setZone(0)

xs = []
ys = []
zs = []
xt = []
yt = []
zt = []
setSpeed(200, 60)

for x in np.linspace(limits[0],limits[1], 2):
    for y in np.linspace(limits[2],limits[3], 2):
        for z in np.linspace(limits[4],limits[5], 1):
            setCart([x,y,z], ori)
            

