#!/usr/bin/env python
# Get an array of 
# [3D apriltag pose from vicon in world frame, apriltag 3D position from detector in webcam frame]

# want to find the mapping between webcam frame wrt world.

from ik.helper import *
from ik.roshelper import *
from apriltags.msg import AprilTagDetections
import tf.transformations as tfm
from rigid_transform_3D import rigid_transform_3D
import numpy as np

pts_vicon = []
pts_apriltag = []

rospy.init_node('robot_vs_webcam')
listener = tf.TransformListener()
for i in xrange(15):
    pause()
    apriltag_pose = lookupTransformList('/map','/apriltag0_vicon', listener)
    pts_vicon.append(apriltag_pose[0:3])
    
    apriltag_detections = ROS_Wait_For_Msg('/apriltags/detections', AprilTagDetections).getmsg()
    for det in apriltag_detections.detections:
        if det.id == 0:
            pts_apriltag.append(pose2list(det.pose)[0:3])
            break

(R,t,rmse) = rigid_transform_3D(np.array(pts_apriltag), np.array(pts_vicon))

data = {}
data["pts_apriltag"] = pts_apriltag
data["pts_vicon"] = pts_vicon
with open('data.txt', 'w') as outfile:
    json.dump(data, outfile)

Rh = tfm.identity_matrix()
Rh[np.ix_([0,1,2],[0,1,2])] = R
quat = tfm.quaternion_from_matrix(Rh)

print 'webcam_T_robot:', " ".join('%.8e' % x for x in (t.tolist() + quat.tolist()))
print 'rmse:', rmse
