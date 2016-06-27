import numpy as np


class Jason():
    def __init__(self):
        self.tool = None
        self.workobject = None

        self.carrier = None
        self.stirrer = None
        self.turntable = None

        self.speed = None
        self.power = None

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

    def path2cmds(self, path):
        commands = []
        commands.append(self.tool)
        commands.append(self.workobject)
        commands.append('{"speed": 50}')
        commands.append('{"laser_main": 1}')
        commands.append('{"laser_standby": 1}')
        commands.append('{"wait_standby": 1}')
        commands.append('{"wait_generalfault": 0}')
        commands.append('{"weldgas": 1}')
        commands.append(self.stirrer)
        commands.append(self.turntable)
        commands.append(self.power)
        commands.append('{"powder_start": 1}')
        commands.append('{"wait_time": 5}')
        commands.append('{"move":[[-25, -25, 125], [1.0, 0.0, 0.0, 0.0]]}')
        commands.append(self.speed)
        for k, pose in enumerate(path):
            cmd = self.get_command_pose(pose)
            commands.append(cmd)
        commands.append(self.get_command_pose([pose[0], pose[1], False]))
        commands.append('{"powder_start":0}')
        commands.append('{"wait_time": 0.5}')
        commands.append('{"weldgas": 0}')
        commands.append('{"speed": 50}')
        commands.append('{"move": [[-25, -25, 125], [1, 0, 0, 0]]}')
        commands.append('{"laser_standby": 0}')
        return commands

    def set_tool(self, tool):
        position = np.float32(tool[0])
        orientation = np.float32(tool[1])
        x, y, z = np.round(position * 1000, 1)
        qx, qy, qz, qw = orientation.round(4)
        self.tool = '{"tool": [[%.1f, %.1f, %.1f], [%.4f, %.4f, %.4f, %.4f]]}' % (x, y, z, qw, qx, qy, qz)

    def set_workobject(self, workobject):
        position = np.float32(workobject[0])
        orientation = np.float32(workobject[1])
        x, y, z = np.round(position * 1000, 1)
        qx, qy, qz, qw = orientation.round(4)
        self.workobject = '{"workobject": [[%.1f, %.1f, %.1f], [%.4f, %.4f, %.4f, %.4f]]}' % (x, y, z, qw, qx, qy, qz)

    def set_powder(self, carrier, stirrer, turntable):
        self.carrier = '{"carrier": %.1f}' % carrier
        self.stirrer = '{"stirrer": %.1f}' % stirrer
        self.turntable = '{"turntable": %.1f}' % turntable

    def set_process(self, speed, power):
        self.speed = '{"speed": %.1f}' % speed
        self.power = '{"power": %.1f}' % power

    def get_command_pose(self, pose):
        x, y, z = np.float32(pose[0]).round(1)
        qx, qy, qz, qw = np.float32(pose[1]).round(4)
        if len(pose) == 3:
            laser = pose[2]
            if laser:
                cmd = '{"move": [[%.1f, %.1f, %.1f], [%.4f, %.4f, %.4f, %.4f], true]}' %(x, y, z, qw, qx, qy, qz)
            else:
                cmd = '{"move": [[%.1f, %.1f, %.1f], [%.4f, %.4f, %.4f, %.4f], false]}' %(x, y, z, qw, qx, qy, qz)
        else:
            cmd = '{"move": [[%.1f, %.1f, %.1f], [%.4f, %.4f, %.4f, %.4f]]}' %(x, y, z, qw, qx, qy, qz)
        return cmd


if __name__ == '__main__':
    routine = Jason()
    path = [[[800, 0, 800], [0, 0, 0, 1]],
            [[1000, 0, 800], [0, 0, 0, 1]],
            [[1000, 0, 1000], [0, 0, 0, 1]]]
    filename = '../../routines/proper.jas'
    routine.set_tool([[215.7, -22.4, 473.8], [0.5, 0.0, -0.8660254, 0.0]])
    routine.set_workobject([[1655, -87, 932], [1, 0, 0, 0]])
    routine.set_powder(3, 20, 20)
    routine.set_process(8, 1000)
    cmds = routine.path2cmds(path)
    routine.save_commands(filename, cmds)
    print cmds
