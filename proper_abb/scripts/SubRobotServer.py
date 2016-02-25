#!/usr/bin/env python
import rospy
from proper_abb.msg import MsgRobotCommand
from abb.server_robot import ServerRobot


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robot_server_listener', anonymous=False)

    rospy.Subscriber('robot_state_chatter', MsgRobotCommand, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
