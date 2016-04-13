#!/usr/bin/env python
import os
import tf
import sys
import rospy
import rospkg
import rosparam
import numpy as np
from std_msgs.msg import String, Header
from proper_abb.srv import SrvRobotCommand
from threading import Thread
from time import sleep
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
        rospy.wait_for_service('robot_send_command')
        self.send_command = rospy.ServiceProxy('robot_send_command',
                                               SrvRobotCommand)
        #self.pub = rospy.Publisher('robot_command_json',
        #                           MsgRobotCommand, queue_size=10)

        # Path buttons
        self.btnLoadPath.clicked.connect(self.btnLoadPathClicked)
        self.btnSavePath.clicked.connect(self.btnSavePathClicked)
        self.btnRunPath.clicked.connect(self.btnRunPathClicked)
        self.btnDelete.clicked.connect(self.btnDeleteClicked)
        self.btnLoadPose.clicked.connect(self.btnLoadPoseClicked)
        self.btnStep.clicked.connect(self.btnStepClicked)
        self.listWidgetPoses.itemDoubleClicked.connect(self.qlistDoubleClicked)
        self.btnCancel.clicked.connect(self.btnCancelClicked)

        self.jason = Jason()
        self.stop = True

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
        '''
        Start-Stop sending commands to robot from a listWidget.
        '''
        #if self.tmrStatus.isActive():
        #    self.tmrStatus.stop()
        #else:
        #    self.tmrStatus.start(10)  # time in ms
        if self.stop:
            self.btnRunPath.setText('Stop')
            self.stop = False
            self.sendPath()
        else:
            self.btnRunPath.setText('Run')
            self.stop = True
            #self.t.join()

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
            #self.pub.publish(item_text.text())
            rob_response = self.send_command(item_text.text())
            print item_text.text()
            row += 1
            if row == n_row:
                row = 0
            self.listWidgetPoses.setCurrentRow(row)

    def btnLoadPoseClicked(self):
        rob_pose = self.send_command('{"get_pose":1}')
        default_command = '{"move":' + rob_pose.response + '}'
        str_command = QtGui.QInputDialog.getText(
            self, "Load Jason Command", "Comamnd:", text=default_command)
        row = self.listWidgetPoses.currentRow()
        if len(str_command[0]) > 3:
            self.insertCommand(str_command[0], insert=True, position=row)
        print str_command

    def qlistDoubleClicked(self):
        row = self.listWidgetPoses.currentRow()
        item_text = self.listWidgetPoses.item(row)
        str_command = QtGui.QInputDialog.getText(
            self, "Load Jason Command", "Comamnd:", text=item_text.text())
        if len(str_command[0]) > 3:
            self.listWidgetPoses.takeItem(row)
            self.insertCommand(str_command[0], insert=True, position=row)

    def btnCancelClicked(self):
        command = '{"cancel":1}'
        rob_response = self.send_command(command)

    def timeStatusEvent(self):
        '''
        Publish one pose from listWidget each time event.
        Not implemented in ros service communication.
        '''
        n_row = self.listWidgetPoses.count()
        if n_row > 0:
            row = self.listWidgetPoses.currentRow()
            if row == -1:
                row = 0
            item_text = self.listWidgetPoses.item(row)
            #self.pub.publish(item_text.text())
            self.tmrStatus.stop()
            rob_response = self.send_command(item_text.text())
            print item_text.text()
            self.tmrStatus.start(10)
            row += 1
            if row == n_row:
                row = 0
                self.tmrStatus.stop()
            self.listWidgetPoses.setCurrentRow(row)

    def sendPath(self):
        '''
        Sends poses from the listWidget until button stop is pressed or all
        poses have been send.
        '''
        n_row = self.listWidgetPoses.count()
        self.ok_command = "OK"
        if n_row > 0:
            while not(self.stop):
                QtGui.QApplication.processEvents()
                sleep(0.1)
                row = self.listWidgetPoses.currentRow()
                if row == -1:
                    row = 0
                #self.pub.publish(item_text.text())
                if self.ok_command == "OK":
                    item_text = self.listWidgetPoses.item(row)
                    self.ok_command = ""
                    row += 1
                    if row == n_row:
                        row = 0
                        self.stop = True
                    else:
                        self.t = Thread(target=self.sendCommand,
                                        args=(item_text.text(),))
                        #self.sendCommand(item_text.text())
                        self.t.start()
                    self.listWidgetPoses.setCurrentRow(row)

    def sendCommand(self, command):
        '''
        Sends command to a service and waits for response.
        '''
        rob_response = self.send_command(command)
        self.ok_command = rob_response.response


if __name__ == "__main__":
    rospy.init_node('path_panel')
    app = QtGui.QApplication(sys.argv)
    qt_path = QtPath()
    qt_path.show()
    app.exec_()
