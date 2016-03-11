#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from abb.logger_robot import LoggerRobot


class PubRobotState():
    def __init__(self):
        self.logger_robot = LoggerRobot()
        rospy.init_node('robot_state', anonymous=False)
        self.pub = rospy.Publisher('joint_states',
                                   JointState, queue_size=10)
        self.msg_joint_state = JointState()
        self.msg_joint_state.name = ['joint_1', 'joint_2', 'joint_3',
                                     'joint_4', 'joint_5', 'joint_6']
        self.msg_joint_state.position = [0, 0, 0, 0, 0, 0]

    def connect(self, ip="172.20.0.32"):
        if rospy.has_param("configuration/robot_ip"):
            ip = rospy.get_param("configuration/robot_ip")
        self.logger_robot.connect(ip)

    def talker(self, j23_coupled=True):
        if rospy.has_param("configuration/J23_coupled"):
            j23_coupled = bool(rospy.get_param("configuration/J23_coupled"))

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.logger_robot.read_logger()
            if len(self.logger_robot.joints) > 0:
                joints_pose = self.logger_robot.joints.popleft()
                for i in range(0, 6):
                    self.msg_joint_state.position[i] = (math.radians(
                        float(joints_pose[i+3])))
                if j23_coupled:
                    self.msg_joint_state.position[2] += (
                        -1 * self.msg_joint_state.position[1])
                self.msg_joint_state.header.stamp = rospy.Time.now()
                #rospy.loginfo(len(logger_robot.joints))
                self.pub.publish(self.msg_joint_state)
                rate.sleep()

    def close_connection(self):
        self.logger_robot.disconnect()

if __name__ == '__main__':
    try:
        robot_publisher = PubRobotState()
        robot_publisher.connect()
        robot_publisher.talker()
        robot_publisher.close_connection()
    except rospy.ROSInterruptException:
        pass
