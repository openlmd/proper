import yaml
import json
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

    def move(self, pose, movel=True):
        self.set_cartesian(pose, linear=movel)

    def speed(self, speed):
        self.set_speed([speed, 500, 50, 50])

    def zone(self, zone):
        self.set_zone(manual_zone=zone)

    def load_file(self, data_file):
        try:
            path_file = open(data_file, 'r')
            for line in path_file:
                self.proc_command(line)
            path_file.close()
        except IOError as e:
            print "I/O error({0}): {1}".format(e.errno, e.strerror)
        except:
            print "Unexpected error"

    def proc_command(self, comando):
        try:
            datos = json.loads(comando)
        except ValueError, e:
            print "Command is not json"
            print e
        else:
            for dato in datos:
                if dato == 'vel':
                    self.speed(datos[dato])
                elif dato == 'pose':
                    self.buffer_add(datos[dato])
                elif dato == 'workobject':
                    self.workobject(datos[dato])
                elif dato == 'tool':
                    self.tool(datos[dato])
                elif dato == 'move':
                    self.move(datos[dato])
                elif dato == 'movej':
                    self.move(datos[dato], movel=False)
                elif dato == 'path_move':
                    if self.buffer_len() > 0:
                        self.buffer_execute()
                elif dato == 'path_clear':
                    self.clear_buffer()
                elif dato == 'path_load':
                    self.load_file(datos[dato])
                else:
                    print 'Dato deconocido: ' + dato
        #if 'pos' in datos:
        #    self.buffer_add(datos['pos'])


if __name__ == '__main__':
    server_robot = ServerRobot()
    server_robot.connect('172.20.0.32')
    # server_robot.workobject([[1.655, -0.087, 0.932], [1, 0, 0, 0]])
    # server_robot.tool([[0.216, -0.022, 0.474], [0.5, 0, -0.866025, 0]])
    # server_robot.speed(50)
    # server_robot.move([[1000, 0, 1000], [0, 0, 1, 0]])
    # server_robot.speed(100)
    # server_robot.move([[900, 0, 900], [0, 0, 1, 0]])
    server_robot.load_file('puntos.txt')
    server_robot.disconnect()
