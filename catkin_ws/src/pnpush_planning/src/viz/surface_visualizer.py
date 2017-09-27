#!/usr/bin/env python

# this is to visualize the robot-arena in rviz

import tf.transformations as tfm
import numpy as np
from ik.roshelper import lookupTransform
import tf
import rospy
from tf.broadcaster import TransformBroadcaster

from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker, createCubeMarker
import sys
from config.surface_db import SurfaceDB 

    
def vizBlock(pose, frame_id, color, scale, marker_id = 1):
    global vizpub
    marker = createCubeMarker( 
                 offset=tuple(pose[0:3]), rgba=color,
                 orientation=tuple(pose[3:7]), frame_id=frame_id, scale = scale, marker_id = marker_id)
    marker.ns = 'surface'
    vizpub.publish(marker)

import optparse
def main(argv):
    parser = optparse.OptionParser()
    parser.add_option('-s', action="store", dest='surface_id', 
                      help='The surface id e.g. abs, delrin, plywood, pu', default='plywood')
    
                      
    (opt, args) = parser.parse_args()
    global vizpub
    rospy.init_node('surface_visulizer')
    listener = tf.TransformListener()
    vizpub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    
    frame_id = '/map'
    r = rospy.Rate(1)
    surface_data = SurfaceDB().db[opt.surface_id]
    while not rospy.is_shutdown():
        vizBlock([surface_data['width']/2+0.06, -0.03, +surface_data['thickness']/2,0,0,0,1], frame_id, surface_data['color'], 
        [surface_data['width'], surface_data['width'], surface_data['thickness']],  marker_id = 120)
        
        r.sleep()

if __name__=='__main__':
    main(sys.argv)
