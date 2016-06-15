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
        if len(work_obj) == 2:
            if (len(work_obj[0]) == 3) and (len(work_obj[1]) == 4):
                self.set_workobject(work_obj)
        else:
            print 'Invalid command format'

    def configure(self, filename):
        print filename

    def move(self, pose, movel=True):
        '''
        Movimiento (lineal si movel==True) a posicion cartesiana.
        El tercer parametro indica estado de trigger.
        '''
        if len(pose) == 2:
            self.set_cartesian(pose, linear=movel)
        elif len(pose) == 3:
            self.set_cartesian_trigg(pose[:2], trigger=pose[2])
        else:
            print 'Invalid command format'

    def move_ext(self, dato):
        '''
        Move external axis (axis, position, speed)
        '''
        if len(dato) == 3:
            self.move_ext_axis(dato[0], dato[1], dato[2])
        else:
            print 'Invalid command format'

    def speed(self, speed):
        self.set_speed([speed, 50, 50, 100])

    def zone(self, zone):
        if len(zone) == 3:
            self.set_zone(manual_zone=zone)
        else:
            print 'Invalid command format'

    def set_digital(self, digital):
        '''
        Dato digital 0 = Valor
        Dato digital 1 = Numero de salida
        '''
        if len(digital) == 2:
            self.set_dio(digital[0], digital[1])
        else:
            print 'Invalid command format'

    def set_analog(self, analog):
        '''
        Dato analogico 0 = Valor
        Dato analogico 1 = Numero de salida
        '''
        if len(analog) == 2:
            analog = list(analog)
            if analog[0] > 100:
                analog[0] = 100
            self.set_ao(analog[0], analog[1])
        else:
            print 'Invalid command format'

    def set_group(self, digital):
        if len(digital) == 2:
            if (type(digital[0]) == int) and (type(digital[1]) == int):
                digital = list(digital)
                if digital[1] == 0:
                    if digital[0] > 31:
                        digital[0] = 31
                if digital[1] == 1:
                    if digital[0] > 65535:
                        digital[0] = 65535
                self.set_gdo(digital[0], digital[1])
        else:
            print 'Invalid command format'

    def buffer_pose(self, pose):
        if len(pose) == 2:
            self.buffer_add(pose)
        elif len(pose) == 3:
            self.buffer_add(pose[:2], True, pose[2])
        else:
            print 'Invalid command format'


if __name__ == '__main__':
    server_robot = ServerRobot()
    server_robot.connect('172.20.0.32')
    # server_robot.workobject([[1.655, -0.087, 0.932], [1, 0, 0, 0]])
    # server_robot.tool([[0.216, -0.022, 0.474], [0.5, 0, -0.866025, 0]])
    # server_robot.speed(50)
    # server_robot.move([[1000, 0, 1000], [0, 0, 1, 0]])
    # server_robot.speed(100)
    # server_robot.move([[900, 0, 900], [0, 0, 1, 0]])
    # server_robot.load_file('puntos.txt')
    server_robot.set_digital(1)
    server_robot.set_analog(50)
    server_robot.disconnect()
