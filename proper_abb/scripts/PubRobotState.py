#!/usr/bin/env python
import rospy
from proper_abb.msg import MsgRobotCommand
from logger_robot import LoggerRobot


def talker():
    pub = rospy.Publisher('joint_state',
                          MsgRobotCommand, queue_size=10)
    rospy.init_node('robot_state', anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():

        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
