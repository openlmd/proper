#!/usr/bin/env python
import os
import tf
import sys
import rospy
import rospkg
import rosparam
import numpy as np
from std_msgs.msg import String, Header
from proper_abb.msg import MsgRobotCommand
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from nav_msgs.msg import Path

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from jason.jason import Jason


path = rospkg.RosPack().get_path('proper_jason')


class QtPath(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'path.ui'), self)

        #self.pub_path = rospy.Publisher('path', String, queue_size=1)
        self.pub = rospy.Publisher('robot_command_json',
                                   MsgRobotCommand, queue_size=10)

        self.btnLoadPath.clicked.connect(self.btnLoadPathClicked)
        icon = QtGui.QIcon.fromTheme('document-open')
        self.btnLoadPath.setIcon(icon)
        self.btnSavePath.clicked.connect(self.btnSavePathClicked)
        icon = QtGui.QIcon.fromTheme('document-save')
        self.btnSavePath.setIcon(icon)
        self.btnRunPath.clicked.connect(self.btnRunPathClicked)
        icon = QtGui.QIcon.fromTheme('media-playback-start')
        self.btnRunPath.setIcon(icon)

        self.btnDelete.clicked.connect(self.btnDeleteClicked)
        self.btnLoadPose.clicked.connect(self.btnLoadPoseClicked)
        self.btnStep.clicked.connect(self.btnStepClicked)

        self.jason = Jason()

        self.tmrStatus = QtCore.QTimer(self)
        self.tmrStatus.timeout.connect(self.timeStatusEvent)

    def insertPose(self, pose):
        (x, y, z), (qx, qy, qz, qw) = pose
        str_pose = '((%.3f, %.3f, %.3f), (%.4f, %.4f, %.4f, %.4f))' %(x, y, z, qx, qy, qz, qw)
        #item = QtGui.QListWidgetItem('item_text')
        #self.listWidgetPoses.addItem(item)
        self.listWidgetPoses.addItem(str_pose)
        #self.listWidgetPoses.insertItem(0, '0, 1, 2')

    def insertCommand(self, command, insert=False, position=0):
        if not insert:
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
        filename = QtGui.QFileDialog.getOpenFileName(
            self, 'Load Path Routine', os.path.join(path, 'routines'),
            'Jason Routine Files (*.jas)')[0]
        print 'Load routine:', filename
        cmds = self.jason.load_commands(filename)
        [self.insertCommand(cmd) for cmd in cmds]

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


if __name__ == "__main__":
    rospy.init_node('path_panel')
    app = QtGui.QApplication(sys.argv)
    qt_path = QtPath()
    qt_path.show()
    app.exec_()
