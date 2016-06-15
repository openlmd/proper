import os
import json


class Jason():
    def __init__(self):
        self.tool = [[0, 0, 0], [1, 0, 0, 0]]
        self.workobject = [[0, 0, 0], [1, 0, 0, 0]]

        self.carrier = 0
        self.stirrer = 0
        self.turntable = 0

        self.speed = 0
        self.power = 0

        self.track_speed = 100
        self.travel_speed = 'v50'
        self.travel_zone = 'z0'

    def path2cmds(self, path):
        cmds = [json.dumps({'tool': self.tool}),
                json.dumps({'workobject': self.workobject}),
                json.dumps({'speed': self.speed}),
                json.dumps({'power': self.power})]
        for k, pose in enumerate(path):
            position, orientation, laser = pose
            x, y, z = position.round(1)
            qx, qy, qz, qw = orientation.round(4)
            print position, orientation, laser
            #cmds.append(json.dumps({'move': [[x, y, z], [qw, qx, qy, qz]]}))
            cmds.append('{"move": [[%.1f, %.1f, %.1f], [%.4f, %.4f, %.4f, %.4f]]}' %(x, y, z, qw, qx, qy, qz))
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

    def set_tool(self, tool):
        self.tool = tool

    def set_workobject(self, workobject):
        self.workobject = workobject

    def set_powder(self, carrier, stirrer, turntable):
        self.carrier = carrier
        self.stirrer = stirrer
        self.turntable = turntable

    def set_process(self, speed, power):
        self.speed = speed
        self.power = power


if __name__ == '__main__':
    routine = Jason()
    path = [[[800, 0, 800], [0, 0, 1, 0]],
            [[1000, 0, 800], [0, 0, 1, 0]],
            [[1000, 0, 1000], [0, 0, 1, 0]]]
    filename = '../../routines/proper.jas'
    routine.set_tool([[215.7, -22.4, 473.8], [0.5, 0.0, -0.8660254, 0.0]])
    routine.set_workobject([[1655, -87, 932], [1, 0, 0, 0]])
    routine.set_powder(3, 20, 20)
    routine.set_process(8, 1000)
    cmds = routine.path2cmds(path)
    routine.save_commands(filename, cmds)
    print cmds
