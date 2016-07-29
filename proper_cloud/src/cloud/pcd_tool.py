import os
import cv2
import numpy as np
import matplotlib.pyplot as plt


def pcd_to_xyz(filename):
    points = np.loadtxt(filename, skiprows=11)
    nfilename = '%s.xyz' % filename[:-4]
    with open(nfilename, 'w') as f:
        np.savetxt(f, points[:, :3], fmt='%.6f')


def xyz_to_pcd(filename):
    points3d = np.loadtxt(filename)
    nfilename = '%s.pcd' % filename[:-4]
    with open(nfilename, 'w') as f:
        f.write('# .PCD v.7 - Point Cloud Data file format\n')
        f.write('VERSION .7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write('WIDTH %i\n' % len(points3d))
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write('POINTS %i\n' % len(points3d))
        f.write('DATA ascii\n')
        np.savetxt(f, points3d, fmt='%.6f')


def read_cloud(filename):
    cloud = np.loadtxt(filename)
    return cloud


def zmap_from_cloud(cloud):
    cloud = cloud[np.all(cloud > 0, axis=1)]
    #point_min = np.min(cloud, axis=0)
    points = (np.round(cloud, 4) * 10000).astype(np.int32)
    x_max = np.max(points[:, 0])
    y_max = np.max(points[:, 1])
    zmap = np.zeros((x_max + 1, y_max + 1))
    zmap[points[:, 0], points[:, 1]] = points[:, 2]
    return zmap


def read_zmap(filename):
    img = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    if len(img.shape) > 2:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def fill_zmap(zmap, size=5):
    kernel = np.ones((size, size), np.uint8)
    closing = cv2.morphologyEx(zmap, cv2.MORPH_CLOSE, kernel)
    return closing


def save_zmap(filename, zmap):
    zmap = zmap.astype(np.uint16)
    cv2.imwrite(filename, zmap)


def show_zmap(zmap):
    plt.figure()
    plt.imshow(zmap, cmap='jet')
    plt.colorbar()
    plt.show()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--data', type=str,
                        default='../../data/downsampled.xyz',
                        help='path to input xyz cloud file')
    args = parser.parse_args()
    filename = args.data

    name, extension = os.path.splitext(filename)
    if extension == '.pcd':
        pcd_to_xyz(filename)
    if extension == '.xyz':
        xyz_to_pcd(filename)

        cloud = read_cloud(filename)
        zmap = zmap_from_cloud(cloud)
        zmap = fill_zmap(zmap, size=7)
        filename = '%s.tif' % name
        save_zmap(filename, zmap)
        show_zmap(zmap)


        img = read_zmap(filename)

        sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=5)
        sobelxy = sobelx + sobely

        plt.figure()
        plt.subplot(2, 2, 1), plt.imshow(img, cmap='gray')
        plt.title('Original'), plt.xticks([]), plt.yticks([])
        plt.subplot(2,2,2),plt.imshow(np.absolute(sobelxy),cmap = 'gray')
        plt.title('Sobel XY'), plt.xticks([]), plt.yticks([])
        plt.subplot(2,2,3),plt.imshow(np.absolute(sobelx),cmap = 'gray')
        plt.title('Sobel X'), plt.xticks([]), plt.yticks([])
        plt.subplot(2,2,4),plt.imshow(np.absolute(sobely),cmap = 'gray')
        plt.title('Sobel Y'), plt.xticks([]), plt.yticks([])
        plt.show()
