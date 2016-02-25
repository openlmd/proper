import abb
from threading import Thread


class LoggerRobot():
    def __init__(self):
        self.robot = abb.Robot()

    def connect(self, ip):
        t = Thread(target=self.robot.connect_logger, args=((ip, 5001), ))
        #self.robot.connect_logger((ip, 5001))
        t.start()

    def disconnect(self):
        pass

if __name__ == '__main__':
    logger_robot = LoggerRobot()
    logger_robot.connect('172.20.0.32')
    logger_robot.disconnect()
