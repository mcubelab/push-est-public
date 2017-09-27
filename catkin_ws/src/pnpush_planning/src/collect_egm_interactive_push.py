#!/usr/bin/env python
import egm_pb2
import socket
from time import sleep
import rospy
import math
from Xlib import display
import Xlib
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
from ik.helper import quat_from_yaw, transformBack, qwxyz_from_qxyzw, transform_back, qxyzw_from_qwxyz
import threading
import tf
from ik.roshelper import pubFrame

from Xlib import X, XK, display
from Xlib.ext import record
from Xlib.protocol import rq

from netft_rdt_driver.srv import Zero

record_dpy = display.Display()
local_dpy = display.Display()

ispause = False
theta = 0.0
flag = True

def lookup_keysym(keysym):
    for name in dir(XK):
        if name[:3] == "XK_" and getattr(XK, name) == keysym:
            return name[3:]
    return "[%d]" % keysym

def smooth_it(src, dst):
    global ispause
    diff = np.array(dst) - np.array(src)
    dist = np.linalg.norm(diff)
    if ispause:
        return src
    if dist > 5:
        newdst = tuple(src + diff / dist * 5)
        return newdst
    return dst
    
def capit(dst):
    dst = list(dst)
    #if dst[0] <= 0.15: dst[0] = 0.15
    #if dst[0] >= 0.35: dst[0] = 0.35
    #if dst[1] <= -0.15: dst[1] = -0.15
    #if dst[1] >= 0.15: dst[1] = 0.15
    return tuple(dst)

def GetTickCount():
    return int((time.time() + 0.5) * 1000)

starttick = GetTickCount()

def record_callback(reply):
    global ispause, theta, flag 
    if reply.category != record.FromServer:
        return
    if reply.client_swapped:
        print "* received swapped protocol data, cowardly ignored"
        return
    if not len(reply.data) or ord(reply.data[0]) < 2:
        # not an event
        return

    data = reply.data
    while len(data):
        event, data = rq.EventField(None).parse_binary_value(data, record_dpy.display, None, None)

        if event.type in [X.KeyPress, X.KeyRelease]:
            pr = event.type == X.KeyPress and "Press" or "Release"

            keysym = local_dpy.keycode_to_keysym(event.detail, 0)
            if not keysym:
                print "KeyCode%s" % pr, event.detail
            else:
                print "KeyStr%s" % pr, lookup_keysym(keysym)
                
                if lookup_keysym(keysym) == 'space' and event.type == X.KeyPress:
                    ispause = not ispause

            if event.type == X.KeyPress and keysym == XK.XK_Escape:
                local_dpy.record_disable_context(ctx)
                local_dpy.flush()
                flag = False
                return
        elif event.type == X.ButtonPress:
            print "ButtonPress", event.detail
            if event.detail == 4:
                theta += 0.05
            if event.detail == 5:
                theta -= 0.05

if __name__=='__main__':
    rospy.init_node('collect_egm_interactive_push')
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size = 2)
    br = tf.TransformBroadcaster()

    setZeroL = rospy.ServiceProxy('/ft_left/zero', Zero)
    setZeroR = rospy.ServiceProxy('/ft_right/zero', Zero)

    def setZero():
        setZeroL()
        setZeroR()
    
    
    rospy.sleep(1)
    setZero()
    
    UDP_PORT=6510
    sequenceNumber = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    
    screen = display.Display().screen()
    root_screen = display.Display().screen().root
    
    # Create a recording context; we only want key and mouse events
    ctx = record_dpy.record_create_context(
            0,
            [record.AllClients],
            [{
                    'core_requests': (0, 0),
                    'core_replies': (0, 0),
                    'ext_requests': (0, 0, 0, 0),
                    'ext_replies': (0, 0, 0, 0),
                    'delivered_events': (0, 0),
                    'device_events': (X.KeyPress, X.MotionNotify),
                    'errors': (0, 0),
                    'client_started': False,
                    'client_died': False,
            }])

    # Enable the context; this only returns after a call to record_disable_context,
    # while calling the callback function in the meantime
    
    t = threading.Thread(target=record_dpy.record_enable_context, args=(ctx, record_callback))
    t.start()
    
    rate = rospy.Rate(250) # 250hz
    while flag:
        rate.sleep()
        # 1. get robot pose
        egm_robot = egm_pb2.EgmRobot()
        data, addr = sock.recvfrom(1024)
        
        if len(data) == 0:
            print("No message\n");
            continue;
            
        egm_robot.ParseFromString(data)
        pos_read = egm_robot.feedBack.cartesian.pos
        orient_read = egm_robot.feedBack.cartesian.orient
        pos_read = pos_read.x,  pos_read.y,  pos_read.z
        #print 'read', pos_read
        print 'orient_read', (orient_read.u0,orient_read.u1,orient_read.u2,orient_read.u3)
        #pos_read = 0.32, 0.03, 0.3

        # 2. if robot connected, do the following
    
        # 3. publish joint position and cartesian position
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js.position = [j for j in egm_robot.feedBack.joints.joints]
        js.velocity = [0.0 for i in xrange(6)]
        js.effort = [0.0 for i in xrange(6)]
        joint_pub.publish(js)
    
        # 4. get mouse location, if delta too far, truncate it.
        qp = root_screen.query_pointer() ; 
        print qp.root_x, qp.root_y
        xd,yd = (qp.root_x-screen.width_in_pixels/2) / 10. * 1.2, -(qp.root_y-screen.height_in_pixels/2) / 10. * 3.
        #xd,yd = 0,0
    
        # 5. send the position
        egm_sensor_write = egm_pb2.EgmSensor()
        
        header = egm_pb2.EgmHeader()
        header.mtype = egm_pb2.EgmHeader.MSGTYPE_CORRECTION
        header.seqno = sequenceNumber
        sequenceNumber += 1
        header.tm = GetTickCount()-starttick
        egm_sensor_write.header.CopyFrom(header)
        
        pos = egm_pb2.EgmCartesian()
        print (300+xd, -30+yd, 230)
        pos.x, pos.y, pos.z = capit(smooth_it(pos_read, (300+xd, -30+yd, 230)))
        orient = egm_pb2.EgmQuaternion()
        q = transform_back([0,0,0,0,1,0,0], [0,0,0]+quat_from_yaw(theta))[3:7]
        qtuple = tuple(qwxyz_from_qxyzw(q))
        #pubFrame(br, [0,0,0] + q)
        
        #print qtuple
        
        orient.u0, orient.u1, orient.u2, orient.u3 = qtuple
        pose = egm_pb2.EgmPose()
        pose.orient.CopyFrom(orient)
        pose.pos.CopyFrom(pos)
        planned = egm_pb2.EgmPlanned()
        planned.cartesian.CopyFrom(pose)
        egm_sensor_write.planned.CopyFrom(planned)
        #print 'write', egm_sensor_write
        sent = sock.sendto(egm_sensor_write.SerializeToString(), addr)
    sock.close()
    print 'End of program'
