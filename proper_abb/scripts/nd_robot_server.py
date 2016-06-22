#!/usr/bin/env python
import json
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
        try:
            command = json.loads(data.command.lower())
            if 'get_pose' in command:
                pose_rob = self.process_command(command)
                return SrvRobotCommandResponse(str(pose_rob))
            self.process_command(command)
            return SrvRobotCommandResponse("OK")
        except ValueError, e:
            print "Command is not json", e
            return SrvRobotCommandResponse("NOK")

    def process_command(self, command):
        for cmd in sorted(command, reverse=True):
            if cmd == 'tool':
                self.server_robot.set_tool(command[cmd])
            elif cmd == 'workobject':
                self.server_robot.workobject(command[cmd])
            elif cmd == 'speed':
                self.server_robot.speed(command[cmd])
            elif cmd == 'power':
                self.server_robot.set_group((11, 0))  # laser program 11
                pwr = int((command[cmd] * 65535) / 1500)  # digital value
                self.server_robot.set_group((pwr, 1))  # laser power
            elif cmd == 'pose':
                self.server_robot.buffer_pose(command[cmd])
            elif cmd == 'move':
                self.server_robot.move(command[cmd])
            elif cmd == 'movej':
                self.server_robot.move(command[cmd], movel=False)
            elif cmd == 'move_ext':
                self.server_robot.move_ext(command[cmd])
            elif cmd == 'path_move':
                if self.server_robot.buffer_len() > 0:
                    self.server_robot.buffer_execute()
            elif cmd == 'path_clear':
                self.server_robot.clear_buffer()
            elif cmd == 'powder_start':
                if command[cmd]:
                    self.server_robot.set_digital((0, 1))  # gtv_stop
                    self.server_robot.set_digital((1, 0))  # gtv_start
                else:
                    self.server_robot.set_digital((1, 1))  # gtv_stop
                    self.server_robot.set_digital((0, 0))  # gtv_start
            elif cmd == 'stirrer':
                stirrer = int((command[cmd] * 100) / 100)  # digital value
                self.server_robot.set_analog((stirrer, 0))  # gtv_disk
            elif cmd == 'turntable':
                turntable = int((command[cmd] * 100) / 10)  # digital value
                self.server_robot.set_analog((turntable, 1))  # gtv_massflow
            elif cmd == 'get_pose':
                return self.server_robot.get_cartesian()
            elif cmd == 'wait_time':
                self.server_robot.wait_time(command[cmd])
            elif cmd == 'wait_standby':
                self.server_robot.wait_input(command[cmd], 0)
            elif cmd == 'wait_generalfault':
                self.server_robot.wait_input(command[cmd], 1)
            elif cmd == 'laser_main':
                self.server_robot.set_digital((command[cmd], 2))
            elif cmd == 'laser_standby':
                self.server_robot.set_digital((command[cmd], 3))
            elif cmd == 'weldgas':
                self.server_robot.set_digital((command[cmd], 4))
            elif cmd == 'rootgas':
                self.server_robot.set_digital((command[cmd], 5))
            elif cmd == 'cancel':
                self.server_robot.cancel_motion()
            else:
                print 'Unknown command:', cmd


if __name__ == '__main__':
    try:
        NdRobotServer()
    except rospy.ROSInterruptException:
        pass
