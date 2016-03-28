import os


class Jason():
    def __init__(self):

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

    def path2cmds(self, path):
        cmds = ['{"tool": %s}' % self.tool,
                '{"workobject": %s}' % self.workobject,
                '{"vel": %i}' % self.track_speed]
        for k, pose in enumerate(path):
            cmds.append('{"move": %s}' % pose)
        return cmds

    def load_commands(self, filename):
        try:
            with open(filename, 'r') as f:
                cmds = [line.rstrip('\n') for line in f.readlines()]
        except IOError:
            cmds = []
        return cmds

    def save_commands(self, filename, cmds):
        try:
            with open(filename, 'w') as f:
                f.writelines('\n'.join(cmds))
        except IOError:
            pass


if __name__ == '__main__':
    routine = Jason()
    path = [[[800, 0, 800], [0, 0, 1, 0]],
            [[1000, 0, 800], [0, 0, 1, 0]],
            [[1000, 0, 1000], [0, 0, 1, 0]]]
    filename = 'proper.jas'
    cmds = routine.path2cmds(path)
    routine.save_commands(filename, cmds)
    print cmds
