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

        #self.marker = MeshMarker(mesh_resource="file://"+filename, frame_id="/workobject")
        self.marker = TriangleListMarker(frame_id='/workobject')
        self.marker.set_points(0.001 * np.vstack(self.robpath.mesh.triangles))
        self.marker.set_color((0.75, 0.25, 0.25, 0.5))
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
        x = 0.001 * self.sbPositionX.value()
        y = 0.001 * self.sbPositionY.value()
        z = 0.001 * self.sbPositionZ.value()
        #self.mesh.translate(position)
        self.marker.set_position((x, y, z))
        #TODO: Change mesh position. Include offset position.
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
        scale = np.float32([sx, sy, sz]) / self.mesh_size
        #TODO: Scale the marker or change the mesh loaded in the mesh.
        self.marker.set_scale(scale=scale)
        self.publisher.publish(self.marker.marker)

    def blockSignals(self, value):
        self.sbPositionX.blockSignals(value)
        self.sbPositionY.blockSignals(value)
        self.sbPositionZ.blockSignals(value)
        self.sbSizeX.blockSignals(value)
        self.sbSizeY.blockSignals(value)
        self.sbSizeZ.blockSignals(value)


#     path = Path()
#     rospy.loginfo(tf.transformations.quaternion_from_euler(0, 0, 0))
#     path.header = Header(frame_id='workobject')
#     #path.header = Header(frame_id='tool0')
#     #path.poses = [PoseStamped(pose=Pose(Point(0, 0, 0.5), Quaternion(0, 0, 0, 1))),
#     #PoseStamped(pose=Pose(Point(0, 1, 1.4), Quaternion(0, 0, 0, 1)))]
#
#     cut_path = (((0, 0.0, 0.0), (0, 0, 0, 1), False),
#                 ((0, 0.0, 0.1), (0, 0, 0, 1), False),
#                 ((0, 0.3, 0.1), (0, 0, 0, 1), False))
#
#     for cut_pose in cut_path:
#         (x, y, z), (q0, q1, q2, q3), proc = cut_pose
#         path.poses.append(PoseStamped(pose=Pose(Point(x/1000, y/1000, z/1000), Quaternion(q0, q1, q2, q3))))
#
#     pub_path.publish(path)
#     rospy.sleep(2.0)
#
#     k = 0
#     N = len(cut_path)
#     while not rospy.is_shutdown() and (k < N):
#         (x, y, z), (q0, q1, q2, q3), proc = cut_path[k]
#         rospy.loginfo("%s, %s" %(cut_path[k], rospy.get_time()))
#         pose = PoseStamped(Header(frame_id='workobject'),
#                            Pose(Point(x/1000, y/1000, z/1000),
#                                 Quaternion(q0, q1, q2, q3)))
#         pub_pose.publish(pose)
#         k = k + 1
#         rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node('part_publisher')

    app = QtGui.QApplication(sys.argv)
    qt_path = QtPart()
    qt_path.show()
    app.exec_()
