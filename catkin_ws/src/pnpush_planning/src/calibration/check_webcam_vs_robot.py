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
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from robot_comm.srv import *
import config.helper as helper

pts_vicon = []
pts_apriltag = []

rospy.init_node('robot_vs_webcam')
listener = tf.TransformListener()

limits = [0.32, 0.54, -0.17, +0.06, 0.24, 0.30]  #[xmin, xmax, ymin, ymax, zmin, zmax]
nseg = [3, 3, 3]
nrotate = 1
ori = [0, 0, 1, 0] # observer
cam_id = 'observer'  
#ori = [0, -0.7071, -0.7071, 0] # viewer
#cam_id = 'viewer'  
globalacc = 2             # some big number means no limit, in m/s^2

setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)
setAcc = rospy.ServiceProxy('/robot2_SetAcc', robot_SetAcc)
setSpeed = rospy.ServiceProxy('/robot2_SetSpeed', robot_SetSpeed)


def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    #print 'setCart', param
    #pause()
    setCartRos(*param)
    
setZone(0)
setSpeed(200, 60)
setAcc(acc=globalacc, deacc=globalacc)

bridge = CvBridge()


data = []
save_dir = os.environ["DATA_BASE"] + "/camera_calib/"

import shutil
shutil.rmtree(save_dir)

helper.make_sure_path_exists(save_dir)

for x in np.linspace(limits[0],limits[1], nseg[0]):
    for y in np.linspace(limits[2],limits[3], nseg[1]):
        for z in np.linspace(limits[4],limits[5], nseg[2]):
            setCart([x,y,z], ori)
            # get robot 3d point
            
            rospy.sleep(0.1)
            cross_poselist = lookupTransformList('/map','/cross_tip', listener)
            
            # take picture of camera
            msg = rospy.wait_for_message("/%s/image_raw" % cam_id, Image)
            timestr = "%.6f" % msg.header.stamp.to_sec()
            image_name = str(save_dir)+timestr+".pgm"
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(image_name, cv_image)
            
            data.append({"cross3d": cross_poselist, "pic_path": timestr+".pgm"})
    
import json
with open(save_dir+'data.json', 'w') as outfile:
    json.dump(data, outfile)


# point3d = [for p["cross3d"] in data]
# point2d = [for p["cross2d"] in data]
