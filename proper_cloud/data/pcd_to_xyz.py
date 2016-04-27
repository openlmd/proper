import sys
import numpy as np


if len(sys.argv) < 2:
    print 'Usage: python xyz_to_pcd.py cloud.pcd'
    sys.exit()
    
filename = sys.argv[1]
points = np.loadtxt(filename, skiprows=11)

nfilename = '%s.xyz' %filename[:-4]
with open(nfilename, 'w') as f:
    np.savetxt(f, points[:,:3], fmt='%.6f')
    
