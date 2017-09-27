#!/usr/bin/env python

import rospy
import numpy as np
import cv2  # OpenCV module

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, String

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

from ik.roshelper import coordinateFrameTransformList
import tf

vis_pub = rospy.Publisher('/visualization_marker_overlay', Marker, queue_size=3) 

fx, fy = 610.0,610.0
cx, cy = 320.0,240.0
zc = 0.6

notagcount = 0
def rosApriltagCallback(msg):
    global notagcount
    print msg.data
    if msg.data == "has_tag":
        notagcount = 0
    elif msg.data == "no_tag":
        notagcount += 1
        
    if notagcount == 0:
        showPyramid(0.1, [1,1,0,0.8], 61)
    elif notagcount > 5:
        showPyramid(0.1, [0,1,1,1], 61)
    
def main():
    
    rospy.init_node('apriltag_cone')
    rospy.Subscriber('/pose_est', String, rosApriltagCallback)
    lr = tf.TransformListener()
    rospy.sleep(1)
    half = 0.016
    top = coordinateFrameTransformList([0, 0, 0], 'observer_rgb_optical_frame', 'map', lr)
    marker_id = 62
    
        
    while not rospy.is_shutdown():
        rgba = [1,1,0,0.8]
        if notagcount == 0:
            rgba = [1,1,0,0.8]
        elif notagcount > 5:
            rgba = [0,1,1,0]
        
        X1 = coordinateFrameTransformList([half, half, 0.01], 'apriltag0_vicon', 'map', lr)
        X2 = coordinateFrameTransformList([half, -half, 0.01], 'apriltag0_vicon', 'map', lr)
        X3 = coordinateFrameTransformList([-half, -half, 0.01], 'apriltag0_vicon', 'map', lr)
        X4 = coordinateFrameTransformList([-half, half, 0.01], 'apriltag0_vicon', 'map', lr)
        vis_pub.publish(createTriangleListMarker(marker_id, [X1, X4, X3, X2], rgba = rgba, frame_id = '/map', o=Point(*top[0:3])))
        rospy.sleep(0.01)
    #rospy.spin()

def getXYZ(xp, yp, zc, fx,fy,cx,cy):
    ## 
    xn = (xp - cx) / fx
    yn = (yp - cy) / fy
    xc = xn * zc
    yc = yn * zc
    print xc, yc
    return (xc,yc,zc)

# Create a pyramid using 4 triangles
def showPyramid(zc, rgba, marker_id):
    # X1-X4 are the 4 corner points of the base of the pyramid
    X1 = getXYZ(0.0, 0.0, zc, fx, fy, cx, cy)
    X2 = getXYZ(640.0, 0.0, zc, fx, fy, cx, cy)
    X3 = getXYZ(640.0, 480.0, zc, fx, fy, cx, cy)
    X4 = getXYZ(0.0, 480.0, zc, fx, fy, cx, cy)
    vis_pub.publish(createTriangleListMarker(marker_id, [X1, X4, X3, X2], rgba = rgba, frame_id = '/observer_rgb_optical_frame'))

def createTriangleListMarker(marker_id, points, rgba, frame_id = '/observer_rgb_optical_frame', o = Point(0,0,0)):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.TRIANGLE_LIST
    marker.scale = Vector3(1,1,1)
    marker.id = marker_id
    
    n = len(points)
    
    if rgba is not None:
        marker.color = ColorRGBA(*rgba)
        
    for i in xrange(n):
        p = Point(*points[i])
        marker.points.append(p)
        p = Point(*points[(i+1)%4])
        marker.points.append(p)
        marker.points.append(o)
        
    marker.pose = poselist2pose([0,0,0,0,0,0,1])
    return marker


def poselist2pose(poselist):
    return Pose(Point(*poselist[0:3]), Quaternion(*poselist[3:7]))

if __name__=='__main__':
    main()
