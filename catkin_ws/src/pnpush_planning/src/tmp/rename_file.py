import glob
import os
filelist = glob.iglob("*/motion*")
for f in filelist:
    if 'v=400' in f:
        os.rename(f, f.replace('v=400', 'v=-1'))
    if 'v=500' in f:
        os.rename(f, f.replace('v=500', 'v=-1'))


