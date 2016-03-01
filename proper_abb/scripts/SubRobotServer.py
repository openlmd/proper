#!/usr/bin/env python
import rospy
from proper_abb.msg import MsgRobotCommand
from abb.server_robot import ServerRobot


class SubRobotServer():
    def __init__(self):
        self.server_robot = ServerRobot()
        rospy.init_node('robot_server', anonymous=False)
        rospy.Subscriber('robot_command_json', MsgRobotCommand, self.callback)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.command)
        self.server_robot.proc_command(data.command)

    def connect(self, ip="172.20.0.32"):
        if rospy.has_param("configuration/robot_ip"):
            ip = rospy.get_param("configuration/robot_ip")
        self.server_robot.connect(ip)

    def listener(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def close_connection(self):
        self.server_robot.disconnect()

if __name__ == '__main__':
    robot_subscriber = SubRobotServer()
    robot_subscriber.connect()
    robot_subscriber.listener()
    robot_subscriber.close_connection()
