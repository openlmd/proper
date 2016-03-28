#!/usr/bin/env python
import os
import os
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


class QtParam(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'param.ui'), self)

        # self.sbShield.valueChanged.connect(self.changeShield)
        # self.sbCarrier.valueChanged.connect(self.changeCarrier)
        # self.sbStirrer.valueChanged.connect(self.changeStirrer)
        # self.sbTurntable.valueChanged.connect(self.changeTurntable)

        self.sbSpeed.valueChanged.connect(self.changeSpeed)
        self.sbPower.valueChanged.connect(self.changePower)

        self.btnAccept.clicked.connect(self.btnAcceptClicked)

    def updateSpeed(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Speed: %s ", data.speed)
        self.lblInfo.setText('Transverse speed: %.3f m/s' % data.speed)

    # TODO: Add event to notify an speed change
    def changeSpeed(self):
        speed = self.sbSpeed.value()
        #self.robpath.set_speed(speed)
        print 'Speed:', speed

    def changePower(self):
        power = self.sbPower.value()
        #self.robpath.set_power(power)
        print 'Power:', power

    def btnAcceptClicked(self):
        print 'Shield:', self.sbShield.value()
        print 'Carrier:', self.sbCarrier.value()
        print 'Stirrer:', self.sbStirrer.value()
        print 'Turntable:', self.sbTurntable.value()
        print 'Speed:', self.sbSpeed.value()
        print 'Power:', self.sbPower.value()



if __name__ == "__main__":
    rospy.init_node('parameters_panel')

    app = QtGui.QApplication(sys.argv)
    qt_param = QtParam()
    qt_param.show()
    app.exec_()
