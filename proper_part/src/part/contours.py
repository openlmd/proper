import cv2
import numpy as np
import matplotlib.pyplot as plt


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
    thrs = np.logical_and(zmap > 100, zmap > 100)
    img_thr[thrs] = 255
    contours, hierarchy = cv2.findContours(
        img_thr.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def approx_contours(contours, epsilon=1):
    contours = [cv2.approxPolyDP(contour, epsilon, True) for contour in contours]
    return contours


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
        points = np.array([[x, y, zmap[y, x]] for x, y in points], np.float)
        slice.append((1. / scale) * points)
    return slice


if __name__ == '__main__':
    import time
    import argparse
    from planning.polyline import filter_polyline
    from planning.planning import Planning
    from planning.mlabplot import MPlot3D

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--data', type=str,
                        default='../../data/pro1.tif',
                        help='path to input zmap image file')
    args = parser.parse_args()
    filename = args.data

    zmap = read_zmap(filename)
    zmap = fill_zmap(zmap, size=7)

    contours = contours_zmap(zmap, thr=100)
    #contours = approx_contours(contours, epsilon=3)
    #contours = [filter_polyline(contour, dist=1) for contour in contours]
    img = draw_contours(zmap, contours, color=(255, 0, 0), size=2)

    #contour = hull_contour(contours, area=1500)
    #img = draw_contours(zmap, [contour], color=(255, 0, 0), size=2)

    plt.figure()
    plt.subplot(121)
    plt.imshow(zmap)
    plt.subplot(122)
    plt.imshow(img)
    plt.show()

    slice = slice_of_contours(contours)

    planning = Planning()

    t0 = time.time()
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
