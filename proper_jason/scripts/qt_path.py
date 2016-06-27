#!/usr/bin/env python
import os
import sys
import json
import rospy
import rospkg
import numpy as np
import tf.transformations as tf

#from proper_abb.msg import MsgRobotCommand
from proper_abb.srv import SrvRobotCommand

from visualization_msgs.msg import MarkerArray
from markers import PathMarkers

from urdf_parser_py.urdf import URDF

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from jason.jason import Jason


path = rospkg.RosPack().get_path('proper_jason')


class QtPath(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'path.ui'), self)

        try:
            rospy.wait_for_service('robot_send_command', timeout=5)
            self.send_command = rospy.ServiceProxy(
                'robot_send_command', SrvRobotCommand)
        except:
            rospy.loginfo('ERROR connecting to service robot_send_command.')
        #self.pub = rospy.Publisher(
        #    import tf'robot_command_json', MsgRobotCommand, queue_size=10)

        self.pub_marker_array = rospy.Publisher(
            'visualization_marker_array', MarkerArray, queue_size=10)

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
        self.btnCancel.clicked.connect(self.btnCancelClicked)

        self.listWidgetPoses.itemSelectionChanged.connect(self.lstPosesClicked)
        self.listWidgetPoses.itemDoubleClicked.connect(self.qlistDoubleClicked)

        self.ok_command = "OK"
        self.path_markers = PathMarkers()

        # Parse robot description file
        robot = URDF.from_parameter_server()
        tcp = robot.joint_map['tcp0']
        workobject = robot.joint_map['workobject']

        tool = [tcp.origin.position,
                list(tf.quaternion_from_euler(*tcp.origin.rotation))]
        print 'Tool:', tool
        workobject = [workobject.origin.position,
                      list(tf.quaternion_from_euler(*workobject.origin.rotation))]
        print 'Workobject:', workobject
        powder = rospy.get_param('/powder')
        print 'Powder:', powder
        process = rospy.get_param('/process')
        print 'Process:', process

        self.jason = Jason()
        self.jason.set_tool(tool)
        self.jason.set_workobject(workobject)
        self.jason.set_powder(
            powder['carrier'], powder['stirrer'], powder['turntable'])
        self.jason.set_process(process['speed'], process['power'])

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

    def loadCommands(self, commands):
        self.listWidgetPoses.clear()
        [self.insertCommand(cmd) for cmd in commands]
        self.arr = []
        self.getMoveCommands()

    def btnLoadPathClicked(self):
        filename = QtGui.QFileDialog.getOpenFileName(
            self, 'Load Path Routine', os.path.join(path, 'routines'),
            'Jason Routine Files (*.jas)')[0]
        print 'Load routine:', filename
        commands = self.jason.load_commands(filename)
        self.loadCommands(commands)

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
        command = json.loads(item_text.text())
        pose = None
        if 'move' in command:
            orientation = np.array([command["move"][1][1],
                                    command["move"][1][2],
                                    command["move"][1][3],
                                    command["move"][1][0]])
            position = np.array(command["move"][0]) * 0.001
            pose = (position, orientation)
        self.path_markers.set_pose(pose)
        self.pub_marker_array.publish(self.path_markers.marker_array)

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
        path = []
        for row in range(n_row):
            item_text = self.listWidgetPoses.item(row)
            command = json.loads(item_text.text())
            if 'move' in command:
                path.append(command['move'])
        self.path_markers.set_path(path)
        self.pub_marker_array.publish(self.path_markers.marker_array)

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
