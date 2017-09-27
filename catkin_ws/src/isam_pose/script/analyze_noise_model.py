#!/usr/bin/env python

import json
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from numpy import array as npa

#prefix = "/home/mcube/pushdata/result/shapeRecon-20170517_142107_246-isam-multipush_shape=ellip2_surface=plywood_rep=0000-ellip2"
prefix = "/home/mcube/pushdata/result/shapeRecon-20170517_145255_041-isam-multipush_shape=butter_surface=plywood_rep=0000-butter"

channel = 'contact'
with open(prefix + "vc_apriltag.json") as jfile:
    vc_apriltag = npa(json.load(jfile))
with open(prefix + "vc_contact.json") as jfile:
    vc_contact = npa(json.load(jfile))
with open(prefix + "vc_push.json") as jfile:
    vc_push = npa(json.load(jfile))
with open(prefix + "vc_static.json") as jfile:
    vc_static = npa(json.load(jfile))

if channel == 'contact':
    vc = vc_contact
    ids = (1,2)
vc = npa([[v[ids[0]], v[ids[1]]] for v in vc if v[0] != 0.0])
#import pdb; pdb.set_trace()
xedges = np.linspace(-0.0015, 0.0015)
yedges = np.linspace(-0.0015, 0.0015)
bins = (xedges, yedges)
H, xedges, yedges = np.histogram2d(vc[:,0], vc[:,1], bins = bins)
H = H.T
print vc.shape

fig = plt.figure()
ax = fig.add_subplot(111, title='vc_contact')
plt.imshow(H, interpolation='nearest', origin='low',
        extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]], cmap='gray')
plt.xlabel('x')
plt.ylabel('y')
plt.show()
