import abb

class LoggerRobot():
    def __init__(self):
        self.robot = abb.Robot()

    def connect(self, ip):
        self.robot.connect_logger((ip, 5001))


if __name__ == '__main__':
    logger_robot = LoggerRobot()
    logger_robot.connect('172.20.0.32')
