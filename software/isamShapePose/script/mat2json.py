#!/usr/bin/env python
import scipy.io as sio
import json
from optparse import OptionParser

def main(args):
    parser = OptionParser(usage="usage: %prog filename_in [filename_out]")
    parser.add_option("-r", "--isreal",
                  action="store_true",
                  dest="isreal",
                  default=False,
                  help="is real or not")
    (options, args) = parser.parse_args()
    
    if len(args) == 2:
        filename_in = args[0]
        filename_out = args[1]
    elif len(args) == 1:
        filename_in = args[0]
        filename_out = filename_in.replace('.mat', '.json')
    else:
        parser.error('no input')
        return 1
    
    # load the mat file
    mat_contents = sio.loadmat(filename_in)
    
    mat_contents['all_contact'] = mat_contents['all_contact'].tolist()
    mat_contents['isreal'] = options.isreal
    mat_contents['__title__'] = [ 'x of contact position',
                                  'y of contact position',
                                  'z of contact position',
                                  'x of contact normal (in simulator) or force (in real)',
                                  'y of contact normal (in simulator) or force (in real)',
                                  'z of contact normal (in simulator) or force (in real)',
                                  'x of ground truth object pose',
                                  'y of ground truth object pose',
                                  'z of ground truth object pose',
                                  'qw of ground truth object pose',
                                  'qx of ground truth object pose',
                                  'qy of ground truth object pose',
                                  'qz of ground truth object pose',
                                  'x of pusher position',
                                  'y of pusher position',
                                  'z of pusher position']
      # d(1:2,:);  % x,y of contact position
      # d(4:5,:);  % x,y contact normal
      # d(7:9,:);  % ground truth box x,y
      # d(10:13,:);  % ground truth box quaternion
      # d(14:16,:);  % pusher position
    with open(filename_out, 'w') as outfile:
        json.dump(mat_contents, outfile, sort_keys=True, indent=1)
    
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
