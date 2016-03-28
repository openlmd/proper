#!/usr/bin/env python
import os
import os
import tf
import sys
import rospy
import rospkg
import rosparam
import numpy as np
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from nav_msgs.msg import Path
from markers import MeshMarker, TriangleListMarker

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from robpath import RobPath


path = rospkg.RosPack().get_path('proper_planning')


class QtPart(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'part.ui'), self)

        self.pub_part = rospy.Publisher('part', String, queue_size=1)
        self.publisher = rospy.Publisher('visualization_marker', Marker, queue_size=1)

        self.btnLoad.clicked.connect(self.btnLoadClicked)
        self.btnProcessMesh.clicked.connect(self.btnProcessMeshClicked)
        self.btnSaveProgram.clicked.connect(self.btnSaveProgramClicked)

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
            self, 'Open file', './', 'Mesh Files (*.stl)')[0]
        print 'Filename:', filename
        self.setWindowTitle(filename)
        self.robpath.load_mesh(filename)

        self.mesh_size = self.robpath.mesh.bpoint2 - self.robpath.mesh.bpoint1
        self.updatePosition(self.robpath.mesh.bpoint1)  # Rename to position
        self.updateSize(self.robpath.mesh.bpoint2 - self.robpath.mesh.bpoint1)

        self.marker = MeshMarker(mesh_resource="file://"+filename, frame_id="/workobject")
        #self.marker = TriangleListMarker()
        #self.marker.set_frame('/workobject')
        #self.marker.set_points(0.001 * np.vstack(self.robpath.mesh.triangles))
        self.marker.set_color((0.9, 0.9, 0.9, 0.6))
#        #rospy.loginfo()
#        self.marker.set_position((0, 0, 0))
#        self.marker.set_scale(scale=(0.001, 0.001, 0.001))
        self.publisher.publish(self.marker.marker)

        #TODO: Change bpoints.
        #self.lblInfo.setText('Info:\n')
        #self.plot.drawMesh(self.robpath.mesh)
        #TODO: Add info from velocity estimation module.

        self.blockSignals(False)

    def updateProcess(self):
        if self.robpath.k < len(self.robpath.levels):
            self.robpath.update_process()
            #self.plot.drawSlice(self.robpath.slices, self.robpath.path)
            #self.plot.progress.setValue(100.0 * self.robpath.k / len(self.robpath.levels))
        else:
            self.processing = False
            self.timer.stop()

    def btnProcessMeshClicked(self):
        if self.processing:
            self.processing = False
            self.timer.stop()
        else:
            height = self.sbHeight.value()
            width = self.sbWidth.value()
            overlap = 0.01 * self.sbOverlap.value()
            self.robpath.set_track(height, width, overlap)

            #self.plot.drawWorkingArea()

            self.robpath.init_process()

            self.processing = True
            self.timer.start(100)

    def btnSaveProgramClicked(self):
        #filename = QtGui.QFileDialog.getOpenFileName(self.plot, 'Save file', './',
        #                                             'Rapid Modules (*.mod)')[0]
        self.robpath.save_rapid()

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
        self.marker.set_points(0.001 * np.vstack(self.robpath.mesh.triangles))
        self.publisher.publish(self.marker.marker)

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
        self.marker.set_points(0.001 * np.vstack(self.robpath.mesh.triangles))
        self.publisher.publish(self.marker.marker)

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
