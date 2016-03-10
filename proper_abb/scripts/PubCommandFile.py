#!/usr/bin/env python
import rospy
from proper_abb.msg import MsgRobotCommand
import os


class PubCommandFile():
    def __init__(self, commandFile):
        rospy.init_node('command_file', anonymous=False)
        self.pub = rospy.Publisher('robot_command_json',
                                   MsgRobotCommand, queue_size=10)
        self.data_file = os.path.abspath(os.path.dirname(__file__))
        self.data_file += "/../src/abb/" + commandFile
        print self.data_file

    def talker(self):
        rate = rospy.Rate(10)  # 10hzrate = rospy.Rate(10)  # 10hz
        try:
            path_file = open(self.data_file, 'r')
            for line in path_file:
                self.pub.publish(line)
                rate.sleep()
            path_file.close()
            rospy.signal_shutdown("File published")
        except IOError as e:
            print "I/O error({0}): {1}".format(e.errno, e.strerror)
        except:
            print "Unexpected error"

if __name__ == '__main__':
    try:
        command_publisher = PubCommandFile("puntos.txt")
        command_publisher.talker()
    except rospy.ROSInterruptException:
        pass
