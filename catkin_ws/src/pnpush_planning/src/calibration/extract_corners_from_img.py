#!/usr/bin/env python


import json
import cv2
import os
import copy
from numpy import array as npa
import numpy as np

save_dir = os.environ["DATA_BASE"] + "/camera_calib/"
with open(save_dir+'data.json') as data_file:    
    data = json.load(data_file)
  
image = []
image_viz = []
refPt = (0,0)
color = (0,255,255)
color2 = (0,0,255)
def click(event, x, y, flags, param):
    global refPt
    global image
    global image_viz
    
    if event == cv2.EVENT_LBUTTONUP:
    # record the ending (x, y) coordinates and indicate that
    # the cropping operation is finished
        refPt = (x, y)
        image_viz = copy.deepcopy(image)
        cv2.line(image_viz, refPt, refPt, color)
        cv2.imshow("image", image_viz)
        print 'clicked point', refPt

def show_updated(pt):
    global image_viz
    print pt
    pt_int = tuple([int(round(p)) for p in pt])
    print pt_int
    cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color2)
    cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color2)
    cv2.imshow("image", image_viz)

newdata = []
for d in data:
    try:
        image = cv2.imread(save_dir + d['pic_path'])
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", click)
        cv2.imshow('image',image)
    except:
        continue
    #(x,y) = getClick()
    while True:
        # display the image and wait for a keypress
        key = cv2.waitKey(3) & 0xFF
        if key == ord("n"):
            break
            
    corners = np.float32([refPt])
    print 'clicked' , refPt
    #import pdb;
    #pdb.set_trace()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    #cv2.cornerSubPix(gray, corners, (10,10), (-1,-1), criteria)
    d["cross2d"] = corners.tolist()[0]
    newdata.append(d)

    # show_updated(tuple(corners.tolist()[0]))

    # while True:
        #display the image and wait for a keypress
        # key = cv2.waitKey(3) & 0xFF
        # if key == ord("n"):
            # break

import json
with open(str(save_dir)+'data.extracted2d.json', 'w') as outfile:
    json.dump(newdata, outfile)
