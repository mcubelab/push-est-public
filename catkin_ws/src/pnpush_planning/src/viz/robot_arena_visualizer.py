#!/usr/bin/env python

# this is to visualize the robot-arena in rviz

import tf.transformations as tfm
import numpy as np
from ik.roshelper import lookupTransform
import tf
import rospy
from tf.broadcaster import TransformBroadcaster

from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker
import sys

def vizBlock(pose, mesh, frame_id):
    global vizpub
    meshmarker = createMeshMarker(mesh, 
                 offset=tuple(pose[0:3]), rgba=(0.5,0.5,0.6,1),
                 orientation=tuple(pose[3:7]), frame_id=frame_id)
    vizpub.publish(meshmarker)

import optparse
def main(argv):
    global vizpub
    rospy.init_node('robot_areana_visulizer')
    listener = tf.TransformListener()
    vizpub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    
    mesh = 'package://pnpush_config/models/robot_arena/robot_arena.stl'
    frame_id = '/map'
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        # get box pose from vicon
        vizBlock([0,0,0,0,0,0,1], mesh, frame_id)
        
        r.sleep()

if __name__=='__main__':
    main(sys.argv)
