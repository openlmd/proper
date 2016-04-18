#!/usr/bin/env python
import os
import tf
import sys
import rospy
import math
import rospkg
import rosparam
import numpy as np
from std_msgs.msg import String, Header
from proper_abb.msg import MsgRobotCommand
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from nav_msgs.msg import Path

from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker, MarkerArray
from markers import LinesMarker
from markers import ArrowMarker


from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from jason.jason import Jason
from transformations import transformations as trans
import json


path = rospkg.RosPack().get_path('proper_jason')


class QtPath(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'path.ui'), self)

        #self.pub_path = rospy.Publisher('path', String, queue_size=1)
        self.pub = rospy.Publisher('robot_command_json',
                                   MsgRobotCommand, queue_size=10)

        # Path buttons
        self.btnLoadPath.clicked.connect(self.btnLoadPathClicked)
        self.btnSavePath.clicked.connect(self.btnSavePathClicked)
        self.btnRunPath.clicked.connect(self.btnRunPathClicked)
        self.btnDelete.clicked.connect(self.btnDeleteClicked)
        self.btnLoadPose.clicked.connect(self.btnLoadPoseClicked)
        self.btnStep.clicked.connect(self.btnStepClicked)
        self.listWidgetPoses.itemSelectionChanged.connect(self.PosesClicked)
        # self.listWidgetPoses.itemChanged.connect(self.getMove)
        self.jason = Jason()
        self.arr = []
        self.offset_position = 100
        self.quat = [0, math.sin(math.radians(45)), 0, math.cos(math.radians(45))]
        self.quat_inv = [0, -math.sin(math.radians(45)), 0, math.cos(math.radians(45))]
        self.marker_array = MarkerArray()
        self.pub_marker_array = rospy.Publisher('visualization_marker_array',
                                                MarkerArray, queue_size=1)

        self.lines = LinesMarker()
        self.lines.set_size(0.005)
        self.lines.set_color((1, 0, 0, 1))
        self.lines.set_frame('/workobject')
        self.marker_array.markers.append(self.lines.marker)

        self.arrow = ArrowMarker(0.1)
        self.arrow.set_color((0, 0, 1, 1))
        self.arrow.set_frame('/workobject')
        self.arrow.set_position((0.2, 0.2, 0.2))
        self.arrow.set_orientation((0, 0, 0, 1))
        self.marker_array.markers.append(self.arrow.marker)

        for id, m in enumerate(self.marker_array.markers):
            m.id = id

        self.tmrStatus = QtCore.QTimer(self)
        self.tmrStatus.timeout.connect(self.timeStatusEvent)

    def insertPose(self, pose):
        (x, y, z), (qx, qy, qz, qw) = pose
        str_pose = '((%.3f, %.3f, %.3f), (%.4f, %.4f, %.4f, %.4f))' %(x, y, z, qx, qy, qz, qw)
        #item = QtGui.QListWidgetItem('item_text')
        #self.listWidgetPoses.addItem(item)
        self.listWidgetPoses.addItem(str_pose)
        #self.listWidgetPoses.insertItem(0, '0, 1, 2')
        self.listWidgetPoses.itemChanged.connect(self.getMove)
nsert:
            self.listWidgetPoses.addItem(command)
        else:
            self.listWidgetPoses.insertItem(position, command)

    def removeComamnd(self):
        item = self.listWidgetPoses.takeItem(0)
        if item:
            print item.text()
            return item.text()
        else:
            return None

    def btnLoadPathClicked(self):
        self.listWidgetPoses.clear()
        filename = QtGui.QFileDialog.getOpenFileName(
            self, 'Load Path Routine', os.path.join(path, 'routines'),
            'Jason Routine Files (*.jas)')[0]
        print 'Load routine:', filename
        cmds = self.jason.load_commands(filename)
        [self.insertCommand(cmd) for cmd in cmds]
        self.arr = []
        self.getMoveCommands()

    def btnSavePathClicked(self):
        filename = QtGui.QFileDialog.getSaveFileName(
            self, 'Load Path Routine', os.path.join(path, 'routines'),
            'Jason Routine Files (*.jas)')[0]
        n_row = self.listWidgetPoses.count()
        if n_row > 0:
            cmds = [str(self.listWidgetPoses.item(row).text()) for row in range(n_row)]
            self.jason.save_commands(filename, cmds)
        print 'Saved routine:', filename

    def btnRunPathClicked(self):
        if self.tmrStatus.isActive():
            self.tmrStatus.stop()
        else:
            self.tmrStatus.start(1000)  # time in ms

    def btnDeleteClicked(self):
        row = self.listWidgetPoses.currentRow()
        self.listWidgetPoses.takeItem(row)
        #self.listWidgetPoses.clear()

    def btnStepClicked(self):
        n_row = self.listWidgetPoses.count()
        if n_row > 0:
            row = self.listWidgetPoses.currentRow()
            if row == -1:
                row = 0
            item_text = self.listWidgetPoses.item(row)
            self.pub.publish(item_text.text())
            print item_text.text()
            row += 1
            if row == n_row:
                row = 0
            self.listWidgetPoses.setCurrentRow(row)

    def btnLoadPoseClicked(self):
        str_command = QtGui.QInputDialog.getText(
            self, "Load Jason Command", "Comamnd:")
        row = self.listWidgetPoses.currentRow()
        row += 1
        self.insertCommand(str_command[0], insert=True, position=row)
        print str_command

    def PosesClicked(self):
        row = self.listWidgetPoses.currentRow()
        item_text = self.listWidgetPoses.item(row)
        str_item = item_text.text()
        comando = json.loads(str_item)
        if 'move' in comando:
            new_arrow = comando["move"][1]
            new_arrow_position = (comando["move"][0])
            new_arrow_position = np. array(new_arrow_position) * 0.001

            self.arrow.set_new_position(new_arrow_position)
            self.arrow.set_new_orientation(new_arrow)
            self.pub_marker_array.publish(self.marker_array)

    def timeStatusEvent(self):
        print 'Timer event'
        n_row = self.listWidgetPoses.count()
        if n_row > 0:
            row = self.listWidgetPoses.currentRow()
            if row == -1:
                row = 0
            item_text = self.listWidgetPoses.item(row)
            self.pub.publish(item_text.text())
            print item_text.text()
            row += 1
            if row == n_row:
                row = 0
                self.tmrStatus.stop()
            self.listWidgetPoses.setCurrentRow(row)

    def getMoveCommands(self):
        n_row = self.listWidgetPoses.count()
        # row = self.listWidgetPoses.currentRow()
        row = 0
        for row in range(0, n_row):
            item_text = self.listWidgetPoses.item(row)
            str_item = item_text.text()
            comando = json.loads(str_item)
            if 'move' in comando:
                new_array_row = comando["move"][0]
                self.arr.append(new_array_row)
        last_array = np. array(self.arr) * 0.001
        self.lines.set_points(last_array)
        self.pub_marker_array.publish(self.marker_array)


if __name__ == "__main__":
    rospy.init_node('path_panel')
    app = QtGui.QApplication(sys.argv)
    qt_path = QtPath()
    qt_path.getMoveCommands()
    qt_path.show()
    app.exec_()
