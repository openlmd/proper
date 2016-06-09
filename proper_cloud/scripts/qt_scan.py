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
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore


path = rospkg.RosPack().get_path('proper_cloud')


class QtScan(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'scan.ui'), self)

        self.btnRecord.clicked.connect(self.btnRecordClicked)

        self.recording = False
        cloud_topic = rospy.get_param('~cloud', '/ueye/cloud')
        rospy.Subscriber(
            cloud_topic, PointCloud2, self.cbPointCloud, queue_size=1)

        self.listener = tf.TransformListener()
        self.filename = '/home/jraraujo/test.xyz'

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

    def cbPointCloud(self, data):
        if self.recording:
            cloud_msg = data
            stamp = data.header.stamp
            points = pc2.read_points(cloud_msg, skip_nans=False)
            points3d = []
            for point in points:
                points3d.append(point)
            points3d = np.float32(points3d)
            #TODO: Record only when the camera is moving.
            points3d = self.point_cloud_to_world(stamp, points3d)
            with open(self.filename, 'a') as f:
                np.savetxt(f, points3d, fmt='%.6f')

    def btnRecordClicked(self):
        if self.recording:
            self.recording = False
            self.btnRecord.setText('Record cloud')
        else:
            try:
                filename = QtGui.QFileDialog.getSaveFileName(
                    self, 'Save file', os.path.join(path, 'data', 'test.xyz'), 'Point Cloud Files (*.xyz)')[0]
                self.filename = filename
                print 'Recording %s ...' % filename
                with open(self.filename, 'w') as f:
                    pass
                self.recording = True
                self.btnRecord.setText('Stop recording...')
            except:
                pass


if __name__ == "__main__":
    rospy.init_node('scan_panel')

    app = QtGui.QApplication(sys.argv)
    qt_scan = QtScan()
    qt_scan.show()
    app.exec_()
