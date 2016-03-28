

class ABB_Robot():
    def __init__(self):
        self.host = '172.20.0.32'
        self.user = 'anonymous'
        self.password = 'anonymous@'

        # Powder conditions
        self.carrier_gas = 3
        self.stirrer = 20
        self.turntable = 20

        self.power = 1200  # define by layer
        self.track_speed = 100

        self.speed = 'vl'  # use speeddata v6
        self.zone = 'z0'

        self.travel_speed = 'v50'
        self.travel_zone = 'z0'

        # Tool pose
        self.tool = [[215.7, -22.4, 473.8], [0.50, 0.0, -0.8660254, 0.0]]
        # Work Object pose
        self.workobject = [[1655, -87, 932], [1, 0, 0, 0]]

    def path2json(self, path_json):
        JSON_TEMPLATE = '{"tool":%(tool)s}\n'
        JSON_TEMPLATE += '{"workobject":%(wobj)s}\n'
        JSON_TEMPLATE += '{"vel":%(speed)i}'
        JSON_TEMPLATE += '%(commands)s\n'
        commands = ''
        for k in range(len(path_json)):
            commands = '\n'.join([commands, path_json[k]])

        tool = self.tool
        wobj = self.workobject

        return JSON_TEMPLATE % {'tool': tool,
                                'wobj': wobj,
                                'speed': self.track_speed,
                                'commands': commands}

    def save_file(self, filename, routine):
        try:
            with open(filename, 'w') as f:
                f.writelines(routine)
        except IOError:
            pass


if __name__ == '__main__':
    rob = ABB_Robot()
    path = ['{"movej":[[800, 0, 800], [0, 0, 1, 0]]}',
            '{"move":[[1000, 0, 800], [0, 0, 1, 0]]}',
            '{"move":[[1000, 0, 1000], [0, 0, 1, 0]]}']
    filename = 'proper.path'
    routine = rob.path2json(path)
    rob.save_file(filename, routine)
    print routine
