#!/usr/bin/env python
import rospy
from proper_abb.srv import SrvRobotCommand, SrvRobotCommandResponse
from abb.server_robot import ServerRobot


class SubRobotServer():
    def __init__(self):
        self.server_robot = ServerRobot()
        rospy.init_node('robot_service_server', anonymous=False)
        self.service = rospy.Service('robot_send_command',
                                     SrvRobotCommand, self.callback)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.command)
        self.server_robot.proc_command(data.command)
        rospy.loginfo(rospy.get_caller_id() + " Processed %s", data.command)
        return SrvRobotCommandResponse("OK")
        #return {'respone': "OK"}

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
    robot_service_handler = SubRobotServer()
    robot_service_handler.connect()
    robot_service_handler.listener()
    robot_service_handler.close_connection()
