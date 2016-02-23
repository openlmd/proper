import yaml
from abb import Robot

class ServerRobot(Robot):
    def __init__(self):
        Robot.__init__(self)

    def connect(self, ip):
        #self.robot.init_ant('172.20.0.32')
        self.connect_motion((ip, 5000))
        self.set_units('millimeters', 'degrees')
        self.set_tool()
        self.set_workobject()
        self.set_speed()
        self.set_zone()

    def disconnect(self):
        self.close()

    def workobject(self, work_obj):
        self.set_workobject(work_obj)

    def tool(self, tool_pose):
        self.set_tool(tool_pose)

    def configure(self, filename):
        print filename

    def move(self, pose):
        self.set_cartesian(pose)

    def speed(self, speed):
        self.set_speed([speed, 500, 50, 50])

    def zone(self, zone):
        self.set_zone(manual_zone=zone)


if __name__ == '__main__':
    server_robot = ServerRobot()
    server_robot.connect('172.20.0.32')
    # server_robot.workobject([[1.655, -0.087, 0.932], [1, 0, 0, 0]])
    # server_robot.tool([[0.216, -0.022, 0.474], [0.5, 0, -0.866025, 0]])
    server_robot.speed(50)
    server_robot.move([[1000, 0, 1000], [0, 0, 1, 0]])
    server_robot.speed(100)
    server_robot.move([[900, 0, 900], [0, 0, 1, 0]])
    server_robot.disconnect()
