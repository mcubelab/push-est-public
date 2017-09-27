

import subprocess, os, signal
import numpy as np
def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)
            
def start_ros_bag(bagfilename, topics, dir_save_bagfile):
    subprocess.Popen('rosbag record -q -O %s %s' % (bagfilename, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)

import os
import errno
def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

def poselist2mat(pose):
    return np.dot(tfm.translation_matrix(pose[0:3]), tfm.quaternion_matrix(pose[3:7]))

def mat2poselist(mat):
    pos = tfm.translation_from_matrix(mat)
    quat = tfm.quaternion_from_matrix(mat)
    return pos.tolist() + quat.tolist()

def getfield_from_filename(figname, field):
    figname = os.path.basename(figname)
    pairs = figname.split('_')
    for p in pairs:
        tmp = p.split('=')
        if tmp[0] == field:
            return tmp[1]
        
    return None

def norm(vect):
    vect = np.array(vect)
    return np.sqrt(np.dot(vect, vect))

global_frame_id = '/map'


def pause():
    print 'Press any key to continue'
    raw_input()
