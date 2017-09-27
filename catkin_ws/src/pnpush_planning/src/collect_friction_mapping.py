#!/usr/bin/env python

# Peter KT Yu, Feb 2016
# use the cage finger to push rect1 to scan the friction of the surface.


import optparse
import rospy
import numpy as np
from config.surface_db import SurfaceDB
from config.helper import pause
import config.helper as helper
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
import os


setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
setZero = rospy.ServiceProxy('/zero', Zero)
setAcc = rospy.ServiceProxy('/robot2_SetAcc', robot_SetAcc)
setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)
setSpeed = rospy.ServiceProxy('/robot2_SetSpeed', robot_SetSpeed)

def wait_for_ft_calib():
    from ik.roshelper import ROS_Wait_For_Msg
    ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()
    
def setCart(pos, ori):
    
    param = (np.array(pos) * 1000).tolist() + ori
    print 'setCart', param
    #pause()
    setCartRos(*param)
    
def main(argv):
    rospy.init_node('collect_friction_map_data')
    
    parser = optparse.OptionParser()
    parser.add_option('', '--surface', action="store", dest='surface_id', 
                      help='The surface id e.g. plywood, abs', default='plywood')
                      
    parser.add_option('', '--cage', action="store_true", dest='tocage', 
                      help='Do the cage process (placing the object inside the cage finger) or not. ', default=False)
    (opt, args) = parser.parse_args()
    
    shape_id = 'rect1'
    ori = [0, 0, 1, 0]
    center_world = [0.375, -0.05, 0]
    z = 0.147-0.01158 + SurfaceDB().db[opt.surface_id]['thickness']
    z_place = z+0.03
    z_up = z_place+0.03
    globalmaxacc = 100
    topics = ['-a']
    skip_when_exists = True
    
    setZone(0)
    # move to center and cage the object
    if opt.tocage:
        setSpeed(tcp=20, ori=1000)
        setAcc(acc=globalmaxacc, deacc=globalmaxacc)
        setCart([center_world[0], center_world[1], z_up], ori)
        
        pause()
        setCart([center_world[0], center_world[1], z_place], ori)
        print 'place the object in the cage with some support and hold it'
        pause()
        setCart([center_world[0], center_world[1], z], ori)
        print 'remove the support'
        pause()
        return
    
    # scanning
    
    max_x = 0.450
    min_x = 0.250
    max_y = 0.197
    min_y = -0.233
    range_x = np.linspace(max_x, min_x, 3)
    acc = 0
    vel = 20
    rep = 0
    nrep = 100
    
    setSpeed(tcp=vel, ori=1000)
     
    dir_save_bagfile = os.environ['PNPUSHDATA_BASE'] + '/friction_scan/%s/%s/' % (opt.surface_id,shape_id)
    helper.make_sure_path_exists(dir_save_bagfile)
    
    for rep in xrange(nrep):
        bagfilename = 'record_surface=%s_shape=%s_a=%.0f_v=%.0f_rep=%03d.bag' % (opt.surface_id, shape_id, acc*1000, vel, rep)
        print bagfilename
        bagfilepath = dir_save_bagfile+bagfilename
        
        if skip_when_exists and os.path.isfile(bagfilepath):
            #print bagfilepath, 'exits', 'skip'
            continue  
        
        setCart([range_x[0], max_y, z], ori)
        setCart([range_x[0], max_y, z_place], ori)
        setZero()
        wait_for_ft_calib()
        #pause()
        setCart([range_x[0], max_y, z], ori)
        
        rosbag_proc = helper.start_ros_bag(bagfilename, topics, dir_save_bagfile)
        for x in range_x:
            setCart([x, max_y, z], ori)
            setCart([x, min_y, z], ori)
            setCart([x, max_y, z], ori)
            
        helper.terminate_ros_node("/record")
        if rep == nrep -1:
            rospy.sleep(1)   # make sure record is terminated completely
    
if __name__=='__main__':
    main(sys.argv)
    


# 231 218.7 123
# 304 -233.3 123
# 466 -233.3 123
# 466 197 123 0 0 -1 0
# 
# setcart 466 197.26 123 0 0 -1 0
# setcart 466 -233.3 123 0 0 -1 0
# setcart 250 -233.3 123 0 0 -1 0
# setcart 250 197.26 123 0 0 -1 0
# 
# setcart 231 197.26 123 0 0 -1 0

