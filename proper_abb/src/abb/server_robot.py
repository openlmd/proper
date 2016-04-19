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

    def speed(self, speed):
        self.set_speed([speed, 500, 5000, 1000])

    def zone(self, zone):
        self.set_zone(manual_zone=zone)

    def set_digital(self, digital):
        '''
        Dato digital 0 = Valor
        Dato digital 1 = Numero de salida
        '''
        self.set_dio(digital[0], digital[1])

    def set_analog(self, digital):
        '''
        Dato analogico 0 = Valor
        Dato analogico 1 = Numero de salida
        '''
        self.set_ao(digital[0], digital[1])

    def set_group(self, digital):
        self.set_gdo(digital[0], digital[1])

    def buffer_pose(self, pose):
        if len(pose) == 2:
            self.buffer_add(pose)
        elif len(pose) == 3:
            self.buffer_add(pose[:2], True, pose[2])

    def proc_command(self, comando):
        '''
        Procesa comandos en formato JSON
        '''
        try:
            comando_json = json.loads(comando.lower())
        except ValueError, e:
            print "Command is not json"
            print e
        else:
            for dato in comando_json:
                if dato == 'vel':
                    self.speed(comando_json[dato])
                elif dato == 'pose':
                    self.buffer_pose(comando_json[dato])
                elif dato == 'workobject':
                    self.workobject(comando_json[dato])
                elif dato == 'tool':
                    self.set_tool(comando_json[dato])
                elif dato == 'move':
                    self.move(comando_json[dato])
                elif dato == 'movej':
                    self.move(comando_json[dato], movel=False)
                elif dato == 'path_move':
                    if self.buffer_len() > 0:
                        self.buffer_execute()
                elif dato == 'path_clear':
                    self.clear_buffer()
                elif dato == 'set_dio':
                    self.set_digital(comando_json[dato])
                elif dato == 'set_ao':
                    self.set_analog(comando_json[dato])
                elif dato == 'laser_prog':
                    program = comando_json[dato]
                    if program > 31:
                        program = 31
                    self.set_group((program, 0))
                elif dato == 'laser_pow':
                    power = comando_json[dato]
                    if power > 65535:
                        power = 65535
                    self.set_group((power, 1))
                elif dato == 'gtv_start':
                    self.set_digital((comando_json[dato], 0))
                elif dato == 'gtv_stop':
                    self.set_digital((comando_json[dato], 1))
                elif dato == 'gtv_disk':
                    self.set_analog((comando_json[dato], 0))
                elif dato == 'gtv_massflow':
                    self.set_analog((comando_json[dato], 1))
                elif dato == 'get_pose':
                    return self.get_cartesian()
                elif dato == 'cancel':
                    self.cancel_motion()
                else:
                    print 'Dato deconocido: ' + dato
        #if 'pos' in comando_json:
        #    self.buffer_add(comando_json['pos'])


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
    server_robot.disconnect()
