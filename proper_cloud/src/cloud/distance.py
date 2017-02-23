import numpy as np
import scipy
from skimage.measure import LineModelND, ransac

def z_max(points3d, pos=False):
    point = 0
    zmax = points3d[0][2]
    for n, p in enumerate(points3d[:]):
        if p[2] > zmax:
            zmax = p[2]
            point = n
    if pos:
        return zmax, point
    else:
        return zmax

def z_min(points3d, pos=False):
    point = 0
    zmin = points3d[0][2]
    for n, p in enumerate(points3d[:]):
        if p[2] < zmin:
            zmin = p[2]
            point = n
    if pos:
        return zmin, point
    else:
        return zmin

def z_heigh_ransac(points3d):
    model_robust, inliers = ransac(points3d, LineModelND, min_samples=2,
                               residual_threshold=0.0001, max_trials=1000)
    outliers = inliers == False
    if len(points3d[outliers]) == 0:
        return 0
    i = 0
    z_avg = 0
    zmax_i = 0
    zmin_i = 0
    zmax = points3d[inliers][0][2]
    zmin = zmax

    for p in points3d[inliers]:
        z_avg = z_avg + p[2]
    z_avg = z_avg / len(points3d[inliers])

    for p_i, p in enumerate(points3d[outliers]):
            if p[2] < zmin:
                zmin = p[2]
                zmin_i = p_i
            if p[2] > zmax:
                zmax = p[2]
                zmax_i = p_i

    #TODO: Comprobar distancia e puntos cercanos (filtrado)
    #k = scipy.spatial.KDTree(points3d[inliers])
    #dist, index = k.query(points3d[outliers][zmax_i])

    z_heigh = zmax - z_avg
    #print 'Distance to model:'
    #print dist
    #print 'Zdiff to model avg:'
    #print z_heigh
    return z_heigh

if __name__ == '__main__':
    pass
