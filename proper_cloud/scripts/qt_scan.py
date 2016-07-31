#!/usr/bin/env python
import os
import sys
import rospy
import rospkg
import numpy as np

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from visualization_msgs.msg import MarkerArray
from markers import ScanMarkers

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from mashes_measures.msg import MsgStatus

from planning.planning import Planning
from cloud.contours import Segmentation
import cloud.pcd_tool as pcd_tool
import cloud.contours as contours


path = rospkg.RosPack().get_path('proper_cloud')


class QtScan(QtGui.QWidget):
    accepted = QtCore.pyqtSignal(list)

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'scan.ui'), self)

        rospy.Subscriber(
            '/ueye/scan', PointCloud2, self.cbPointCloud, queue_size=5)
        rospy.Subscriber(
            '/supervisor/status', MsgStatus, self.cbStatus, queue_size=1)

        self.pub_marker_array = rospy.Publisher(
            'visualization_marker_array', MarkerArray, queue_size=10)

        self.btnPlane.clicked.connect(self.btnPlaneClicked)
        self.btnRecord.clicked.connect(self.btnRecordClicked)
        self.btnZmap.clicked.connect(self.btnZmapClicked)
        self.btnScan.clicked.connect(self.btnScanClicked)

        self.sbPositionX.valueChanged.connect(self.changePosition)
        self.sbPositionY.valueChanged.connect(self.changePosition)
        self.sbPositionZ.valueChanged.connect(self.changePosition)

        self.sbSizeX.valueChanged.connect(self.changeSize)
        self.sbSizeY.valueChanged.connect(self.changeSize)
        self.sbSizeZ.valueChanged.connect(self.changeSize)

        self.filename = ''
        self.status = False
        self.running = False
        self.recording = False

        self.path = []
        self.scan_markers = None
        self.planning = Planning()
        self.position = np.array([0, 0, 10])
        self.size = np.array([100, 200, 0])

    def cbPointCloud(self, msg_cloud):
        if self.recording:
            points = pc2.read_points(msg_cloud, skip_nans=False)
            points3d = np.float32([point for point in points])
            print self.filename
            with open(self.filename, 'a') as f:
                np.savetxt(f, points3d, fmt='%.6f')

    def cbStatus(self, msg_status):
        status = msg_status.running
        if not self.status and status:
            if self.running:
                self.recording = True
                self.btnRecord.setText('Recording...')
        elif self.status and not status:
            self.running = False
            self.recording = False
            self.btnRecord.setText('Record Cloud')
        self.status = status

    def updatePlane(self):
        (x, y, z), (w, h, t) = self.position, self.size
        plane = np.array([[x, y, z], [x+w, y, z], [x+w, y+h, z], [x, y+h, z]])
        slice = [plane - np.array([0, 50, 0])]  # laser stripe offset
        path = self.planning.get_path_from_slices(
            [slice], track_distance=100, focus=100)
        self.path = [[pos, ori] for pos, ori, bol in path]
        self.scan_markers.set_plane_size(self.size)
        self.scan_markers.set_plane_position(self.position)
        self.scan_markers.set_path(self.path)
        self.pub_marker_array.publish(self.scan_markers.marker_array)

    def changePosition(self):
        self.position = np.array([self.sbPositionX.value(),
                                  self.sbPositionY.value(),
                                  self.sbPositionZ.value()])
        self.updatePlane()

    def changeSize(self):
        self.size = np.array([self.sbSizeX.value(),
                              self.sbSizeY.value(),
                              self.sbSizeZ.value()])
        self.updatePlane()

    def btnPlaneClicked(self):
        self.scan_markers = ScanMarkers()
        self.updatePlane()

    def btnRecordClicked(self):
        if self.running:
            self.running = False
            self.btnRecord.setText('Record Cloud')
        else:
            try:
                filename = QtGui.QFileDialog.getSaveFileName(
                    self, 'Save file', os.path.join(path, 'data', 'test.xyz'),
                    'Point Cloud Files (*.xyz)')[0]
                self.filename = filename
                with open(self.filename, 'w') as f:
                    pass
                self.running = True
                self.btnRecord.setText('Stop recording...')
            except:
                pass

    def btnZmapClicked(self):
        filename = QtGui.QFileDialog.getOpenFileName(
            self, 'Load file', os.path.join(path, 'data', 'test.xyz'),
            'Point Cloud Files (*.xyz)')[0]
        name, extension = os.path.splitext(filename)
        cloud = pcd_tool.read_cloud(filename)
        zmap = pcd_tool.zmap_from_cloud(cloud)
        zmap = pcd_tool.fill_zmap(zmap, size=7)
        pcd_tool.save_zmap('%s.tif' % name, zmap)
        segmentation = Segmentation()
        segmentation.plot_zmap(zmap)
        slice = contours.slice_of_contours(segmentation.contours)
        print slice

    def btnScanClicked(self):
        self.accepted.emit(self.path)


if __name__ == "__main__":
    rospy.init_node('scan_panel')

    app = QtGui.QApplication(sys.argv)
    qt_scan = QtScan()
    qt_scan.show()
    app.exec_()
