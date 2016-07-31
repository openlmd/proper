#!/usr/bin/env python
import os
import sys
import rospy
import rospkg
import numpy as np

from visualization_msgs.msg import MarkerArray
from markers import PartMarkers

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from planning.robpath import RobPath


path = rospkg.RosPack().get_path('proper_part')


class QtPart(QtGui.QWidget):
    accepted = QtCore.pyqtSignal(list)

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'part.ui'), self)

        self.pub_marker_array = rospy.Publisher(
            'visualization_marker_array', MarkerArray, queue_size=10)

        self.btnLoad.clicked.connect(self.btnLoadClicked)
        self.btnProcessMesh.clicked.connect(self.btnProcessMeshClicked)
        self.btnLayers.clicked.connect(self.btnLayersClicked)
        self.btnAcceptPath.clicked.connect(self.btnAcceptPathClicked)

        self.sbStart.valueChanged.connect(self.changeLayers)
        self.sbStop.valueChanged.connect(self.changeLayers)

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
        self.updateParameters()

    def btnLoadClicked(self):
        self.blockSignals(True)
        self.processing = False

        filename = QtGui.QFileDialog.getOpenFileName(
            self, 'Open file', os.path.join(path, 'data'),
            'Mesh Files (*.stl)')[0]
        print 'Filename:', filename
        self.setWindowTitle(filename)
        self.robpath.load_mesh(filename)

        #TODO: Add info from velocity estimation module.
        self.mesh_size = self.robpath.mesh.size
        self.updatePosition(self.robpath.mesh.position)
        self.updateSize(self.robpath.mesh.size)

        self.part_markers = PartMarkers()
        self.part_markers.set_mesh(self.robpath.mesh)
        self.pub_marker_array.publish(self.part_markers.marker_array)

        self.updateLayers()

        self.blockSignals(False)

    def updateParameters(self):
        height = self.sbHeight.value()
        width = self.sbWidth.value()
        overlap = 0.01 * self.sbOverlap.value()
        print height, width, overlap
        self.robpath.set_track(height, width, overlap)
        self.robpath.set_process(rospy.get_param('/process/speed'),
                                 rospy.get_param('/process/power'),
                                 rospy.get_param('/process/focus'))

    def updateLayers(self):
        self.npoints = 0
        levels = self.robpath.init_process()
        self.sbStart.setValue(self.robpath.k)
        self.sbStop.setValue(len(levels))

    def updateProcess(self):
        if self.robpath.k < len(self.robpath.levels):
            path = self.robpath.update_process(filled=self.chbFill.isChecked(),
                                               contour=self.chbContour.isChecked())
            self.sbStart.setValue(self.robpath.k)
            print 'Path:', path
            path = self.robpath.path[self.npoints:-1]
            self.npoints = self.npoints + len(path)
            self.part_markers.set_path(path)
            self.pub_marker_array.publish(self.part_markers.marker_array)
        else:
            self.processing = False
            self.timer.stop()

    def btnProcessMeshClicked(self):
        if self.processing:
            self.processing = False
            self.timer.stop()
        else:
            self.updateParameters()
            self.updateLayers()
            self.processing = True
            self.timer.start(100)

    def btnLayersClicked(self):
        if self.processing:
            self.updateParameters()
            self.updateProcess()
        else:
            self.updateParameters()
            self.updateLayers()
            self.processing = True

    def btnAcceptPathClicked(self):
        self.accepted.emit(self.robpath.path)

    def changeLayers(self):
        start = self.sbStart.value()
        stop = self.sbStop.value()
        self.robpath.k = int(start)

    def updatePosition(self, position):
        x, y, z = position
        self.sbPositionX.setValue(x)
        self.sbPositionY.setValue(y)
        self.sbPositionZ.setValue(z)

    def changePosition(self):
        x = self.sbPositionX.value()
        y = self.sbPositionY.value()
        z = self.sbPositionZ.value()
        self.updatePart(position=(x, y, z))

    def updateSize(self, size):
        sx, sy, sz = size
        self.sbSizeX.setValue(sx)
        self.sbSizeY.setValue(sy)
        self.sbSizeZ.setValue(sz)

    def changeSize(self):
        sx = self.sbSizeX.value()
        sy = self.sbSizeY.value()
        sz = self.sbSizeZ.value()
        self.updatePart(size=(sx, sy, sz))

    def updatePart(self, position=None, size=None):
        if position is not None:
            self.robpath.translate_mesh(np.float32(position))
            self.part_markers.set_mesh(self.robpath.mesh)
        if size is not None:
            self.robpath.resize_mesh(np.float32(size))
            self.part_markers.set_mesh(self.robpath.mesh)
        self.pub_marker_array.publish(self.part_markers.marker_array)

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
