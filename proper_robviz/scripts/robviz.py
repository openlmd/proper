#!/usr/bin/env python
import os
import sys
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

import rviz

from mashes_measures.msg import MsgVelocity

from qt_scan import QtScan
from qt_param import QtParam
from qt_part import QtPart
from qt_path import QtPath


path = rospkg.RosPack().get_path('proper_robviz')


class MyViz(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()

        reader.readFile(config, os.path.join(path, 'config', 'workcell.rviz'))
        self.frame.load(config)

        self.setWindowTitle(config.mapGetChild("Title").getValue())

        self.frame.setMenuBar(None)
        self.frame.setHideButtonVisibility(False)

        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

        layout = QtGui.QVBoxLayout()
        layout.setContentsMargins(9, 0, 9, 0)
        self.setLayout(layout)

        h_layout = QtGui.QHBoxLayout()
        layout.addLayout(h_layout)

        orbit_button = QtGui.QPushButton("Orbit View")
        orbit_button.clicked.connect(self.onOrbitButtonClick)
        h_layout.addWidget(orbit_button)

        front_button = QtGui.QPushButton("Front View")
        front_button.clicked.connect(self.onFrontButtonClick)
        h_layout.addWidget(front_button)

        right_button = QtGui.QPushButton("Rigth View")
        right_button.clicked.connect(self.onRightButtonClick)
        h_layout.addWidget(right_button)

        top_button = QtGui.QPushButton("Top View")
        top_button.clicked.connect(self.onTopButtonClick)
        h_layout.addWidget(top_button)

        layout.addWidget(self.frame)

    def switchToView(self, view_name):
        view_man = self.manager.getViewManager()
        for i in range(view_man.getNumViews()):
            if view_man.getViewAt(i).getName() == view_name:
                view_man.setCurrentFrom(view_man.getViewAt(i))
                return
        print("Did not find view named %s." % view_name)

    def onOrbitButtonClick(self):
        self.switchToView("Orbit View")

    def onFrontButtonClick(self):
        self.switchToView("Front View")

    def onRightButtonClick(self):
        self.switchToView("Right View")

    def onTopButtonClick(self):
        self.switchToView("Top View")


class RobPathUI(QtGui.QMainWindow):
    def __init__(self):
        super(RobPathUI, self).__init__()
        loadUi(os.path.join(path, 'resources', 'robviz.ui'), self)

        self.boxPlot.addWidget(MyViz())

        self.qtScan = QtScan()
        self.qtParam = QtParam()
        self.qtPart = QtPart()
        self.qtPath = QtPath()

        self.tabWidget.addTab(self.qtScan, 'Scan')
        self.tabWidget.addTab(self.qtParam, 'Params')
        self.tabWidget.addTab(self.qtPart, 'Part')
        self.tabWidget.addTab(self.qtPath, 'Path')

        self.qtParam.accepted.connect(self.qtParamAccepted)
        self.qtPart.accepted.connect(self.qtPartAccepted)

        self.btnQuit.clicked.connect(self.btnQuitClicked)
        icon = QtGui.QIcon.fromTheme('application-exit')
        self.btnQuit.setIcon(icon)

        rospy.Subscriber(
            '/velocity', MsgVelocity, self.cbVelocity, queue_size=1)

    def cbVelocity(self, msg_velocity):
        self.lblInfo.setText("Speed: %.1f mm/s" % (1000 * msg_velocity.speed))

    def qtParamAccepted(self):
        self.tabWidget.setCurrentWidget(self.qtPart)

    def qtPartAccepted(self, path):
        commands = self.qtPath.jason.path2cmds(path)
        self.qtPath.loadCommands(commands)
        self.tabWidget.setCurrentWidget(self.qtPath)

    def btnQuitClicked(self):
        QtCore.QCoreApplication.instance().quit()


if __name__ == '__main__':
    rospy.init_node('robviz')

    app = QtGui.QApplication(sys.argv)
    robpath = RobPathUI()
    robpath.show()
    app.exec_()
