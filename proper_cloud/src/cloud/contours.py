import cv2
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from matplotlib.widgets import SpanSelector
from matplotlib.patches import Rectangle


def read_zmap(filename):
    img = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    if len(img.shape) > 2:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def fill_zmap(zmap, size=5):
    kernel = np.ones((size, size), np.uint8)
    closing = cv2.morphologyEx(zmap, cv2.MORPH_CLOSE, kernel)
    return closing


def contours_zmap(zmap, thr=100):
    img_thr = np.zeros(zmap.shape, np.uint8)
    if type(thr) is tuple:
        thrs = np.logical_and(zmap > thr[0], zmap < thr[1])
    else:
        thrs = zmap > thr
    img_thr[thrs] = 255
    contours, hierarchy = cv2.findContours(
        img_thr.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def approx_contours(contours, epsilon=1):
    return [cv2.approxPolyDP(contour, epsilon, True) for contour in contours]


def draw_contours(zmap, contours, color=(255, 0, 0), size=None):
    if zmap.dtype == np.uint16:
        img = (zmap / 256).astype(np.uint8)
    else:
        img = zmap.copy()
    if size is None:
        size = int(np.round(float(img.shape[0]) / 512))
    if len(img.shape) < 3 and type(color) != int:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(img, contours, -1, color, size)
    return img


def hull_contour(contours, area=1000):
    contour = np.vstack(contours)
    if cv2.contourArea(contour) > area:
        contour = cv2.convexHull(contour)
        contour = cv2.approxPolyDP(contour, 1.0, True)
    return contour


def slice_of_contours(contours, scale=10):
    slice = []
    for contour in contours:
        points = contour.reshape((-1, 2))
        points = [[x, y, zmap[y, x]] for x, y in points]
        points.append(points[0])
        points = np.array(points, np.float)
        slice.append((1. / scale) * points)
    return slice


class Segmentation:
    def __init__(self):
        pass

    def on_select(self, xmin, xmax):
        print xmin, xmax
        xmin = int(round(xmin))
        xmax = int(round(xmax))
        self.rect.set_xy((xmin, 0))
        self.rect.set_width(xmax-xmin)
        if xmin == xmax:
            self.ax1.imshow(zmap)
        else:
            self.contours = contours_zmap(zmap, thr=(xmin, xmax))
            img = draw_contours(zmap, self.contours, color=(255, 0, 0), size=2)
            self.ax1.imshow(img)
        self.fig.canvas.draw()

    def plot_zmap(self, zmap):
        gs = gridspec.GridSpec(5, 1)
        self.fig = plt.figure(figsize=(8, 6))
        self.ax1 = self.fig.add_subplot(gs[:4, :])
        self.ax1.imshow(zmap)
        self.ax1.axes.get_xaxis().set_visible(False)
        self.ax1.axes.get_yaxis().set_visible(False)
        ax2 = self.fig.add_subplot(gs[4, :])
        ax2.hist(zmap.flatten(), 100, range=(1, zmap.max()), fc='k', ec='k')
        ax2.axes.get_yaxis().set_visible(False)
        height = ax2.get_ylim()[1]
        self.rect = Rectangle((0, 0), 0, height, alpha=.2, color=(1, 0, 0))
        ax2.add_patch(self.rect)
        span = SpanSelector(ax2, self.on_select, 'horizontal', useblit=True,
                            rectprops=dict(alpha=0.5, facecolor='red'))
        plt.show()


if __name__ == '__main__':
    import time
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--data', type=str,
                        default='../../data/pro1.tif',
                        help='path to input zmap image file')
    parser.add_argument('-t', '--threshold', type=int,
                        default=100, help='threshold value for contours')
    args = parser.parse_args()
    filename = args.data
    threshold = args.threshold

    zmap = read_zmap(filename)
    zmap = fill_zmap(zmap, size=7)

    contours = contours_zmap(zmap, thr=threshold)
    #contours = approx_contours(contours, epsilon=3)
    img = draw_contours(zmap, contours, color=(255, 0, 0), size=2)

    #contour = hull_contour(contours, area=1500)
    #img = draw_contours(zmap, [contour], color=(255, 0, 0), size=2)

    plt.figure()
    plt.subplot(121)
    plt.imshow(zmap)
    plt.subplot(122)
    plt.imshow(img)
    plt.show()

    segmentation = Segmentation()
    segmentation.plot_zmap(zmap)
    #contours = approx_contours(segmentation.contours, epsilon=2)
    #contours = [hull_contour(segmentation.contours, area=1000)]
    slice = slice_of_contours(segmentation.contours)

    from planning.planning import Planning
    from planning.mlabplot import MPlot3D
    from planning.polyline import filter_polyline

    t0 = time.time()
    planning = Planning()
    slice = [filter_polyline(polyline, dist=0.3) for polyline in slice]
    path = planning.get_path_from_slices([slice], 0.8)
    t1 = time.time()
    print 'Time for path:', t1 - t0

    # # Get path with frames
    # _path = []
    # for position, orientation, process in path:
    #    frame, t = calc.quatpose_to_pose(position, orientation)
    #    _path.append([position, frame, process])

    mplot3d = MPlot3D()
    mplot3d.draw_slice(slice)
    mplot3d.draw_path(path)
    mplot3d.show()
