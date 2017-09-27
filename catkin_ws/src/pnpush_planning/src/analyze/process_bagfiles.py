#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# main function to parse a folder of bagfiles 

import subprocess
import sys, os
import glob
import optparse

def main(argv):
    parser = optparse.OptionParser()
    parser.add_option('', '--noplotmotion', action="store_false", dest='plotmotion', 
                      help='Not do the plotting. ', default=True)
    parser.add_option('', '--plotmotion', action="store_true", dest='plotmotion', 
                      help='Do the plotting. ', default=True)
    parser.add_option('', '--noplotfmap', action="store_false", dest='plotfmap', 
                      help='Not do the friction map plotting. ', default=False)
    parser.add_option('', '--plotfmap', action="store_true", dest='plotfmap', 
                      help='Do the friction map plotting. ', default=False)
    parser.add_option('', '--avgcolorbar', action="store", type='float', dest='avgcolorbar', 
                      help='Color bar', nargs=2, default=(0,1))
    parser.add_option('', '--norri', action="store_false", dest='rri', 
                      help='No RRI ', default=True)
  
    (opt, args) = parser.parse_args()
                   
        
    dirname = args[0]
    filelist = sorted(glob.glob("%s/*.bag" % dirname))
    
    plotmotion = opt.plotmotion
    plotfmap = opt.plotfmap
    
    nthread = 7
    
    if opt.rri:
        rri = ''
    else:
        rri = '--norri' 
    for i, bag_filepath in enumerate(filelist):
        if not os.path.exists(bag_filepath.replace('bag','json')):
            proc = subprocess.Popen('rosrun pnpush_planning parse_bagfile_to_rawjson.py %s --json %s' % (bag_filepath, rri) , shell=True)
            if i % nthread == nthread-1:
                proc.wait()
    
    for i, bag_filepath in enumerate(filelist):            
        if plotmotion and not os.path.exists(bag_filepath.replace('bag','png')):
            proc = subprocess.Popen('rosrun pnpush_planning plot_raw_h5.py %s snapshots' % (bag_filepath.replace('.bag', '.h5')) , shell=True)
            if i % nthread == nthread-1:
                proc.wait()

    for i, bag_filepath in enumerate(filelist):  
        if plotfmap and not os.path.exists(bag_filepath.replace('.bag','_fmap.png')):
            proc = subprocess.Popen('rosrun pnpush_planning plot_friction_map.py %s --avgcolorbar %f %f' % 
                       (bag_filepath.replace('.bag', '.h5'), opt.avgcolorbar[0], opt.avgcolorbar[1]) , shell=True)
            if i % nthread == nthread-1:
                proc.wait()

if __name__=='__main__':
    main(sys.argv)
    
