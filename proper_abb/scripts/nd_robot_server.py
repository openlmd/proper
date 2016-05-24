#!/usr/bin/env python
import rospy
from proper_abb.srv import SrvRobotCommand
from proper_abb.srv import SrvRobotCommandResponse
from abb.server_robot import ServerRobot


class NdRobotServer():
    def __init__(self):
        rospy.init_node('robot_service_server', anonymous=False)

        self.service = rospy.Service(
            'robot_send_command', SrvRobotCommand, self.cb_robot_command)

        robot_ip = rospy.get_param('~robot_ip', '192.168.30.4')
        self.server_robot = ServerRobot()
        self.server_robot.connect(robot_ip)

        rospy.on_shutdown(self.server_robot.disconnect)
        rospy.spin()

    def cb_robot_command(self, data):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.command)
        if data.command[2:10] == 'get_pose':
            pose_rob = self.server_robot.proc_command(data.command)
            return SrvRobotCommandResponse(str(pose_rob))
        self.server_robot.proc_command(data.command)
        return SrvRobotCommandResponse("OK")


if __name__ == '__main__':
    try:
        NdRobotServer()
    except rospy.ROSInterruptException:
        pass
