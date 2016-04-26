#!/usr/bin/env python
import os
import sys
import rospy
import rospkg
import numpy as np
#from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from markers import MeshMarker
from markers import LinesMarker
from markers import TriangleListMarker

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from robpath import RobPath


path = rospkg.RosPack().get_path('proper_planning')


class QtPart(QtGui.QWidget):
    accepted = QtCore.pyqtSignal(list)

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'part.ui'), self)

        self.pub_marker_array = rospy.Publisher(
            'visualization_marker_array', MarkerArray, queue_size=1)

        self.btnLoad.clicked.connect(self.btnLoadClicked)
        self.btnProcessMesh.clicked.connect(self.btnProcessMeshClicked)
        self.btnProcessLayer.clicked.connect(self.btnProcessLayerClicked)
        self.btnAcceptPath.clicked.connect(self.btnAcceptPathClicked)

        self.sbPositionX.valueChanged.connect(self.changePosition)
        self.sbPositionY.valueChanged.connect(self.changePosition)
        self.sbPositionZ.valueChanged.connect(self.changePosition)

        self.sbSizeX.valueChanged.connect(self.changeSize)
        self.sbSizeY.valueChanged.connect(self.changeSize)
        self.sbSizeZ.valueChanged.connect(self.changeSize)

        self.processing = False
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateProcess)

        self.robpath = RobPath()

    def btnLoadClicked(self):
        self.blockSignals(True)

        filename = QtGui.QFileDialog.getOpenFileName(
            self, 'Open file', os.path.join(path, 'data'), 'Mesh Files (*.stl)')[0]
        print 'Filename:', filename
        self.setWindowTitle(filename)
        self.robpath.load_mesh(filename)

        self.mesh_size = self.robpath.mesh.bpoint2 - self.robpath.mesh.bpoint1
        self.updatePosition(self.robpath.mesh.bpoint1)  # Rename to position
        self.updateSize(self.robpath.mesh.bpoint2 - self.robpath.mesh.bpoint1)

        self.marker_array = MarkerArray()
        #self.marker = MeshMarker(mesh_resource="file://"+filename, frame_id="/workobject")
        self.mesh = TriangleListMarker()
        self.mesh.set_frame('/workobject')
        self.mesh.set_points(0.001 * np.vstack(self.robpath.mesh.triangles))
        self.mesh.set_color((0.66, 0.66, 0.99, 0.66))
        self.marker_array.markers.append(self.mesh.marker)
        self.path = LinesMarker()
        self.path.set_frame('/workobject')
        self.path.set_color((1.0, 0.0, 0.0, 1.0))
        self.marker_array.markers.append(self.path.marker)
        for id, m in enumerate(self.marker_array.markers):
            m.id = id
        self.npoints = 0
#        #rospy.loginfo()
        self.pub_marker_array.publish(self.marker_array)

        #TODO: Change bpoints.
        #self.lblInfo.setText('Info:\n')
        #self.plot.drawMesh(self.robpath.mesh)
        #TODO: Add info from velocity estimation module.

        self.blockSignals(False)

    def updateParameters(self):
        height = self.sbHeight.value()
        width = self.sbWidth.value()
        overlap = 0.01 * self.sbOverlap.value()
        print height, width, overlap
        self.robpath.set_track(height, width, overlap)

    def updateProcess(self):
        if self.robpath.k < len(self.robpath.levels):
            self.robpath.update_process()
            #self.plot.drawSlice(self.robpath.slices, self.robpath.path)
            points = np.array([pose[0] for pose in self.robpath.path[self.npoints:-1]])
            self.npoints = self.npoints + len(points)
            self.path.set_points(0.001 * points)
            print len(points)
            self.pub_marker_array.publish(self.marker_array)
            #self.plot.progress.setValue(100.0 * self.robpath.k / len(self.robpath.levels))
        else:
            self.processing = False
            self.timer.stop()

    def btnProcessMeshClicked(self):
        if self.processing:
            self.processing = False
            self.timer.stop()
        else:
            self.updateParameters()
            self.robpath.init_process()
            self.processing = True
            self.timer.start(100)

    def btnProcessLayerClicked(self):
        if self.processing:
            self.updateParameters()
            self.updateProcess()
        else:
            self.updateParameters()
            self.robpath.init_process()
            self.processing = True

    def btnAcceptPathClicked(self):
        self.accepted.emit(self.robpath.path)

    def updatePosition(self, position):
        x, y, z = position
        self.sbPositionX.setValue(x)
        self.sbPositionY.setValue(y)
        self.sbPositionZ.setValue(z)

    def changePosition(self):
        x = self.sbPositionX.value()
        y = self.sbPositionY.value()
        z = self.sbPositionZ.value()
        self.robpath.translate_mesh(np.float32([x, y, z]))
        #self.marker.set_position((x, y, z))
        self.mesh.set_points(0.001 * np.vstack(self.robpath.mesh.triangles))
        self.pub_marker_array.publish(self.marker_array)

    def updateSize(self, size):
        sx, sy, sz = size
        self.sbSizeX.setValue(sx)
        self.sbSizeY.setValue(sy)
        self.sbSizeZ.setValue(sz)

    def changeSize(self):
        sx = self.sbSizeX.value()
        sy = self.sbSizeY.value()
        sz = self.sbSizeZ.value()
        self.robpath.resize_mesh(np.float32([sx, sy, sz]))
        #scale = np.float32([sx, sy, sz]) / self.mesh_size
        self.mesh.set_points(0.001 * np.vstack(self.robpath.mesh.triangles))
        self.pub_marker_array.publish(self.marker_array)

    def blockSignals(self, value):
        self.sbPositionX.blockSignals(value)
        self.sbPositionY.blockSignals(value)
        self.sbPositionZ.blockSignals(value)
        self.sbSizeX.blockSignals(value)
        self.sbSizeY.blockSignals(value)
        self.sbSizeZ.blockSignals(value)


if __name__ == "__main__":
    rospy.init_node('part_panel')

    app = QtGui.QApplication(sys.argv)
    qt_part = QtPart()
    qt_part.show()
    app.exec_()
