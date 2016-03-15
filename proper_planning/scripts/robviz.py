#!/usr/bin/env python
import os
import sys
import rospy
import rospkg
import rosparam

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

import rviz

from geometry_msgs.msg import Pose2D

import tf
import numpy as np
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
#from mashes_measures.msg import MsgVelocity

from qt_path import QtPath
from qt_part import QtPart


path = rospkg.RosPack().get_path('proper_planning')


class MyViz(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application. In this example, we disable everything
        ## so that the only thing visible is the 3D render window.
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        # Read the configuration from the config file for visualization.
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

    ## switchToView() works by looping over the views saved in the
    ## ViewManager and looking for one with a matching name.
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

        # TODO: other buttons
        self.btnRecord.clicked.connect(self.btnRecordClicked)

        # Add tabs
        self.tabWidget.addTab(QtGui.QPushButton('Tab X'), 'Tab X')
        self.tabWidget.addTab(QtPath(), 'Path')
        self.tabWidget.addTab(QtPart(), 'Part')

        # Process buttons
        self.sbSpeed.valueChanged.connect(self.changeSpeed)
        self.sbPower.valueChanged.connect(self.changePower)

        self.btnQuit.clicked.connect(self.btnQuitClicked)

        self.recording = False
        cloud_topic = rospy.get_param('~cloud', '/camera/cloud')
        rospy.Subscriber(cloud_topic, PointCloud2, self.callback_point_cloud, queue_size=1)

        self.listener = tf.TransformListener()

        # Timer
        self.tmrInfo = QtCore.QTimer(self)
        self.tmrInfo.timeout.connect(self.timeInfoEvent)
        self.tmrInfo.start(100)

        # Subscribers
        #rospy.Subscriber('velocity', MsgVelocity, self.updateSpeed)

    def timeInfoEvent(self):
        #cmd = self._detach_command()
        #if not cmd == None:
        #    if self.alasHead.client.send_command(cmd) == 0:
        #        self._insert_command(cmd)
        print self.lblInfo.text()


    def updateSpeed(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Speed: %s ", data.speed)
        self.lblInfo.setText('Transverse speed: %.3f m/s' % data.speed)

    def changeSpeed(self):
        speed = self.Window.sbSpeed.value()
        self.robpath.set_speed(speed)

    def changePower(self):
        power = self.Window.sbPower.value()
        self.robpath.set_power(power)


    def point_cloud_to_world(self, stamp, points3d):
        """Transforms the point cloud in camera coordinates to the world frame."""
        self.listener.waitForTransform("/world", "/camera0", stamp, rospy.Duration(1.0))
        (position, quaternion) = self.listener.lookupTransform("/world", "/camera0", stamp)
        matrix = tf.transformations.quaternion_matrix(quaternion)
        matrix[:3, 3] = position
        points = np.zeros((len(points3d), 3), dtype=np.float32)
        for k, point3d in enumerate(points3d):
            point = np.ones(4)
            point[:3] = point3d
            points[k] = np.dot(matrix, point)[:3]
        return points

    def callback_point_cloud(self, data):
        if self.recording:
            cloud_msg = data
            stamp = data.header.stamp
            points = pc2.read_points(cloud_msg, skip_nans=False)
            points3d = []
            for point in points:
                points3d.append(point)
            points3d = np.float32(points3d)
            rospy.loginfo(points3d)
            #TODO: Record only when the camera is moving.
            points3d = self.point_cloud_to_world(stamp, points3d)
            with open('test.xyz', 'a') as f:
                np.savetxt(f, points3d, fmt='%.6f')

    def btnRecordClicked(self):
        if self.recording:
            print 'Stop record'
            self.recording = False
        else:
            print 'Recording...'
            with open('test.xyz', 'w') as f:
                pass
            self.recording = True

    def btnQuitClicked(self):
        self.tmrInfo.stop()
        QtCore.QCoreApplication.instance().quit()


if __name__ == '__main__':
    rospy.init_node('robviz')

    app = QtGui.QApplication(sys.argv)
    robpath = RobPathUI()
    robpath.show()
    app.exec_()
