#!/usr/bin/env python
import os
import sys
import json
import rospy
import rospkg
import numpy as np
#from proper_abb.msg import MsgRobotCommand
from proper_abb.srv import SrvRobotCommand
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray

from markers import LinesMarker
from markers import ArrowMarker

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from jason.jason import Jason


path = rospkg.RosPack().get_path('proper_jason')


class QtPath(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'path.ui'), self)

        rospy.wait_for_service('robot_send_command')
        self.send_command = rospy.ServiceProxy(
            'robot_send_command', SrvRobotCommand)
        #self.pub = rospy.Publisher(
        #    'robot_command_json', MsgRobotCommand, queue_size=10)

        self.btnLoadPath.clicked.connect(self.btnLoadPathClicked)
        self.btnSavePath.clicked.connect(self.btnSavePathClicked)
        self.btnRunPath.clicked.connect(self.btnRunPathClicked)
        self.btnDelete.clicked.connect(self.btnDeleteClicked)
        self.btnLoadPose.clicked.connect(self.btnLoadPoseClicked)
        self.btnStep.clicked.connect(self.btnStepClicked)
        self.btnCancel.clicked.connect(self.btnCancelClicked)

        self.listWidgetPoses.itemSelectionChanged.connect(self.lstPosesClicked)
        self.listWidgetPoses.itemDoubleClicked.connect(self.qlistDoubleClicked)

        self.jason = Jason()
        self.ok_command = "OK"

        self.offset_position = 100
        self.quat = [0, np.sin(np.deg2rad(45)), 0, np.cos(np.deg2rad(45))]
        self.quat_inv = [0, -np.sin(np.deg2rad(45)), 0, np.cos(np.deg2rad(45))]

        self.pub_marker_array = rospy.Publisher(
            'visualization_marker_array', MarkerArray, queue_size=10)

        self.marker_array = MarkerArray()

        self.lines = LinesMarker()
        self.lines.set_size(0.005)
        self.lines.set_color((1, 0, 0, 1))
        self.lines.set_frame('/workobject')
        self.marker_array.markers.append(self.lines.marker)

        self.arrow = ArrowMarker(0.1)
        self.arrow.set_color((0, 0, 1, 1))
        self.arrow.set_frame('/workobject')
        # self.arrow.set_position((0.2, 0.2, 0.2))
        # self.arrow.set_orientation((0, 0, 0, 1))
        self.marker_array.markers.append(self.arrow.marker)

        for id, m in enumerate(self.marker_array.markers):
            m.id = id

        self.tmrRunPath = QtCore.QTimer(self)
        self.tmrRunPath.timeout.connect(self.timeRunPathEvent)

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
        """Start-Stop sending commands to robot from the list of commands."""
        if self.tmrRunPath.isActive():
            self.tmrRunPath.stop()
            self.btnRunPath.setText('Run')
        else:
            self.btnRunPath.setText('Stop')
            self.tmrRunPath.start(100)  # time in ms

    def btnDeleteClicked(self):
        row = self.listWidgetPoses.currentRow()
        self.listWidgetPoses.takeItem(row)
        #self.listWidgetPoses.clear()

    def btnLoadPoseClicked(self):
        rob_pose = self.send_command('{"get_pose":1}')
        default_command = '{"move":' + rob_pose.response + '}'
        str_command = QtGui.QInputDialog.getText(
            self, "Load Jason Command", "Comamnd:", text=default_command)
        row = self.listWidgetPoses.currentRow()
        if len(str_command[0]) > 3:
            self.insertCommand(str_command[0], insert=True, position=row)
        print str_command

    def btnStepClicked(self):
        n_row = self.listWidgetPoses.count()
        if n_row > 0:
            row = self.listWidgetPoses.currentRow()
            if row == -1:
                row = 0
            item_text = self.listWidgetPoses.item(row)
            #self.pub.publish(item_text.text())
            self.sendCommand(item_text.text())
            row += 1
            if row == n_row:
                row = 0
            self.listWidgetPoses.setCurrentRow(row)

    def lstPosesClicked(self):
        row = self.listWidgetPoses.currentRow()
        item_text = self.listWidgetPoses.item(row)
        str_item = item_text.text()
        command = json.loads(str_item)
        if 'move' in command:
            new_arrow = command["move"][1]
            new_arrow_position = (command["move"][0])
            new_arrow_position = np.array(new_arrow_position) * 0.001
            self.arrow.set_new_position(new_arrow_position)
            self.arrow.set_new_orientation(new_arrow)
            self.arrow.set_color((1, 0, 0, 1))
        else:
            self.arrow.set_color((0, 0, 0, 0))
        self.pub_marker_array.publish(self.marker_array)

    def qlistDoubleClicked(self):
        row = self.listWidgetPoses.currentRow()
        item_text = self.listWidgetPoses.item(row)
        str_command = QtGui.QInputDialog.getText(
            self, "Load Jason Command", "Comamnd:", text=item_text.text())
        if len(str_command[0]) > 3:
            self.listWidgetPoses.takeItem(row)
            self.insertCommand(str_command[0], insert=True, position=row)

    def btnCancelClicked(self):
        self.sendCommand('{"cancel":1}')

    def getMoveCommands(self):
        n_row = self.listWidgetPoses.count()
        # row = self.listWidgetPoses.currentRow()
        points = []
        for row in range(n_row):
            item_text = self.listWidgetPoses.item(row)
            str_item = item_text.text()
            comando = json.loads(str_item)
            if 'move' in comando:
                point = comando["move"][0]
                points.append(point)
        points = np.array(points) * 0.001
        print points
        self.lines.set_points(points)
        self.pub_marker_array.publish(self.marker_array)

    def sendCommand(self, command):
        rob_response = self.send_command(command)
        print 'Sended command:', command
        print 'Received response:', rob_response
        self.ok_command = rob_response.response

    def timeRunPathEvent(self):
        """Sends a command each time event from the list of commands."""
        n_row = self.listWidgetPoses.count()
        if n_row > 0:
            row = self.listWidgetPoses.currentRow()
            if row == -1:
                row = 0
            item_text = self.listWidgetPoses.item(row)
            #self.pub.publish(item_text.text())
            self.sendCommand(item_text.text())
            if self.ok_command == "OK":
                row += 1
                if row == n_row:
                    row = 0
                    self.btnRunPathClicked()
                self.listWidgetPoses.setCurrentRow(row)


if __name__ == "__main__":
    rospy.init_node('path_panel')
    app = QtGui.QApplication(sys.argv)
    qt_path = QtPath()
    qt_path.show()
    app.exec_()
