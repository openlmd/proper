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
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from nav_msgs.msg import Path

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore


path = rospkg.RosPack().get_path('proper_planning')


class QtPath(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'path.ui'), self)

        self.pub_path = rospy.Publisher('path', String, queue_size=1)

        # Path buttons
        self.btnLoadPath.clicked.connect(self.btnLoadPathClicked)
        self.btnSavePath.clicked.connect(self.btnSavePathClicked)
        self.btnRunPath.clicked.connect(self.btnRunPathClicked)
        self.btnStatus.clicked.connect(self.btnStatusClicked)

        self.tmrStatus = QtCore.QTimer(self)
        self.tmrStatus.timeout.connect(self.timeStatusEvent)
        self.tmrStatus.start(1000)  # time in ms

    def insertPose(self, pose):
        (x, y, z), (qx, qy, qz, qw) = pose
        str_pose = '((%.3f, %.3f, %.3f), (%.4f, %.4f, %.4f, %.4f))' %(x, y, z, qx, qy, qz, qw)
        #item = QtGui.QListWidgetItem('item_text')
        #self.listWidgetPoses.addItem(item)
        self.listWidgetPoses.addItem(str_pose)
        #self.listWidgetPoses.insertItem(0, '0, 1, 2')

    def removePose(self):
        item = self.listWidgetPoses.takeItem(0)
        if item:
            print item.text()
            return item.text()
        else:
            return None

    def btnLoadPathClicked(self):
        filename = QtGui.QFileDialog.getOpenFileName(
            self, 'Load Path Routine', './', 'Path Routine Files (*.path)')[0]
        print 'Load path:', filename
        #file = open(filename, 'r')
        #cmds = file.readlines()
        #for cmd in cmds:
        #    self._append_command(cmd)
        #self._append_command_file(self.filename)

    def btnSavePathClicked(self):
        print 'Save path'

    def btnRunPathClicked(self):
        print 'Run path'
        pose = np.array([[0.150, 0.100, 0.200], [1.0, 0.1, 0.1, 0.1]])
        self.insertPose(pose)

    def btnStatusClicked(self):
        print 'Button pressed'
        self.pub_path.publish(String('[1, 2, 3]'))

    def timeStatusEvent(self):
        print 'Timer event'
        path = String('[0, 0, 0]')
        self.pub_path.publish(path)


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
    rospy.init_node('path_publisher')

    app = QtGui.QApplication(sys.argv)
    qt_path = QtPath()
    qt_path.show()
    app.exec_()
