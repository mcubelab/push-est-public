#!/usr/bin/env python

# this is to visualize the block in rviz

import tf.transformations as tfm
import numpy as np
from ik.roshelper import lookupTransform
import tf
import rospy
from tf.broadcaster import TransformBroadcaster

from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker, createSphereMarker
import sys

from config.shape_db import ShapeDB

def vizBlock(vizpub, pose, mesh, frame_id, rgba=(0.5,0.5,0.5,1), marker_id=2):
    # prepare block visualization
    meshmarker = createMeshMarker(mesh, 
                 offset=tuple(pose[0:3]), rgba=rgba,
                 orientation=tuple(pose[3:7]), frame_id=frame_id, marker_id = marker_id)
    meshmarker.ns = frame_id
    vizpub.publish(meshmarker)

def vizSphere(vizpub, pose, scale, frame_id, rgba=(0.5,0.5,0.5,1), marker_id=2):
    # prepare block visualization
    meshmarker = createSphereMarker(color=rgba,
                 scale = scale,
                 offset=tuple(pose[0:3]),  
                 orientation=tuple(pose[3:7]), frame_id=frame_id, marker_id = marker_id)
    meshmarker.ns = frame_id
    vizpub.publish(meshmarker)


import optparse
def main(argv):
    parser = optparse.OptionParser()
    parser.add_option('-s', action="store", dest='shape_id', 
                      help='The shape id e.g. rect1-rect3', default='rect1')
    
                      
    (opt, args) = parser.parse_args()
    vicon_ball_size = 0.01415  # in diameter

    rospy.init_node('vicon_object_visulizer')
    listener = tf.TransformListener()
    vizpub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    vizpuboverlay = rospy.Publisher("visualization_marker_overlay", Marker, queue_size=10)
    
    
    # parameters about object
    shape_id = opt.shape_id  # should be a rosparam
    shape_db = ShapeDB()
    mesh = shape_db.shape_db[shape_id]['mesh']
    frame_id = shape_db.shape_db[shape_id]['frame_id']
    obj_slot = shape_db.shape_db[shape_id]['slot_pos']
    thickness = shape_db.shape_db[shape_id]['thickness']
    vicon_marker_plate_thick = 0.002
    
    obj_des_wrt_vicon = [0,0,-(thickness/2 + vicon_ball_size + vicon_marker_plate_thick) ,0,0,0,1]  # from vicon to the block (a slight difference in z)
    obj_des_wrt_vicon_h = [0,0,-(thickness/2 + vicon_ball_size + vicon_marker_plate_thick)+0.001 ,0,0,0,1]  # from vicon to the block (a slight difference in z)
    
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        # get box pose from vicon
        vizSphere(vizpuboverlay, [0,0,0,0,0,0,1], [0.001,0.001,0.001], 'cross_tip', (0,1,1,1), marker_id=24)
        try:
            vizBlock(vizpub, obj_des_wrt_vicon, mesh, frame_id)
            vizBlock(vizpuboverlay, obj_des_wrt_vicon_h, mesh, 'estimate', (1,0,0,0.9), marker_id=22)
            vizBlock(vizpuboverlay, obj_des_wrt_vicon_h, mesh, 'estimate_EKF', (0,1,0,0.9), marker_id=25)
            vizBlock(vizpuboverlay, obj_des_wrt_vicon, mesh, 'apriltag', (0,1,1,0.9), marker_id=23)
        except:
            print 'object not in view'
        
        r.sleep()

if __name__=='__main__':
    main(sys.argv)
