#!/usr/bin/python

import rospy
import xml.dom.minidom
from sensor_msgs.msg import JointState
from math import pi
from threading import Thread

RANGE = 10000

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class JointStatePublisher():
    def __init__(self):
        description = get_param('robot_description')
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})
        use_mimic = get_param('use_mimic_tags', True)
        use_small = get_param('use_smallest_joint_limits', True)

        self.zeros = get_param("zeros")

        pub_def_positions = get_param("publish_default_positions", True)
        pub_def_vels = get_param("publish_default_velocities", False)
        pub_def_efforts = get_param("publish_default_efforts", False)

        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if use_small and len(safety_tags)==1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if use_mimic and len(mimic_tags)==1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval}
                if pub_def_positions:
                    joint['position'] = zeroval
                if pub_def_vels:
                    joint['velocity'] = 0.0
                if pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint

        source_list = get_param("source_list", [])
        self.sources = []
        for source in source_list:
            self.sources.append(rospy.Subscriber(source, JointState, self.source_cb))

        self.pub = rospy.Publisher('joint_states', JointState)

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self.free_joints:
                continue

            if msg.position:
                position = msg.position[i]
            else:
                position = None
            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None
            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.free_joints[name]
            if position is not None:
                joint['position'] = position
            if velocity is not None:
                joint['velocity'] = velocity
            if effort is not None:
                joint['effort'] = effort

        if self.gui is not None:
            # post an event here instead of directly calling the update_sliders method, to switch to the wx thread
            wx.PostEvent(self.gui.GetEventHandler(), self.gui.UpdateSlidersEvent())


    def loop(self):
        hz = get_param("rate", 10) # 10hz
        r = rospy.Rate(hz)

        delta = get_param("delta", 0.0)

        # Publish Joint States
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()

            if delta > 0:
                self.update(delta)

            # Initialize msg.position, msg.velocity, and msg.effort.
            has_position = len(self.dependent_joints.items()) > 0
            has_velocity = False
            has_effort = False
            for (name,joint) in self.free_joints.items():
                if not has_position and 'position' in joint:
                    has_position = True
                if not has_velocity and 'velocity' in joint:
                    has_velocity = True
                if not has_effort and 'effort' in joint:
                    has_effort = True
            num_joints = (len(self.free_joints.items()) +
                          len(self.dependent_joints.items()))
            if has_position:
                msg.position = num_joints * [0.0]
            if has_velocity:
                msg.velocity = num_joints * [0.0]
            if has_effort:
                msg.effort = num_joints * [0.0]


            for i, name in enumerate(self.joint_list):
                msg.name.append(str(name))
                joint = None

                # Add Free Joint
                if name in self.free_joints:
                    joint = self.free_joints[name]
                    factor = 1
                    offset = 0
                # Add Dependent Joint
                elif name in self.dependent_joints:
                    param = self.dependent_joints[name]
                    parent = param['parent']
                    joint = self.free_joints[parent]
                    factor = param.get('factor', 1)
                    offset = param.get('offset', 0)

                if has_position and 'position' in joint:
                    msg.position[i] = joint['position'] * factor + offset
                if has_velocity and 'velocity' in joint:
                    msg.velocity[i] = joint['velocity'] * factor
                if has_effort and 'effort' in joint:
                    msg.effort[i] = joint['effort']

            self.pub.publish(msg)
            r.sleep()

    def update(self, delta):
        for name, joint in self.free_joints.iteritems():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    if joint.get('continuous', False):
                        joint['position'] = joint['min']
                    else:
                        joint['position'] = joint['max']
                        joint['forward'] = not forward
            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward


if __name__ == '__main__':
    try:
        rospy.init_node('joint_state_publisher')
        jsp = JointStatePublisher()

        if jsp.gui is None:
            jsp.loop()
        else:
            Thread(target=jsp.loop).start()
            jsp.app.MainLoop()
    except rospy.ROSInterruptException:
        pass


# #! /usr/bin/env python
# import roslib; roslib.load_manifest('abb_irb_5400')
#
# import sys
#
# import rospy
# import tf
#
# from sensor_msgs.msg import JointState
#
# from moveit_msgs.srv import GetPositionIK
# from moveit_msgs.msg import PositionIKRequest
# from geometry_msgs.msg import PoseStamped
#
#
# if __name__ == '__main__':
#     rospy.init_node('test_position_ik')
#
#     pub_joint_states = rospy.Publisher('joint_states', JointState)
#
#     rospy.wait_for_service('compute_ik')
#     get_position_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
#
#     target = PoseStamped()
#     target.header.frame_id = 'base_link'
#     x, y, z = 1.5, 0.6, 1.5
#     target.pose.position.x = x
#     target.pose.position.y = y
#     target.pose.position.z = z
#     qx, qy, qz, qw = 0.0, -0.3, 0.0, 0.954
#     target.pose.orientation.x = qx
#     target.pose.orientation.y = qy
#     target.pose.orientation.z = qz
#     target.pose.orientation.w = qw
#
#     service_request = PositionIKRequest()
#     service_request.group_name = 'irb_5400'
#     #service_request.robot_state = initial_state
#     #service_request.ik_link_name = 'link_6'
#     service_request.pose_stamped = target
#     service_request.timeout.secs= 0.1
#     service_request.avoid_collisions = False
#
#     rospy.loginfo("Request = {0}".format(service_request))
#
#     resp = get_position_ik(service_request)
#     rospy.loginfo("Response = {0}".format(resp))
#
#     #rospy.loginfo("Base position = [{0},{1},{2}".format(resp.solution.joint_state.position[0],resp.solution.joint_state.position[1],resp.solution.joint_state.position[2]))
#     position = resp.solution.joint_state.position
#     rospy.loginfo("Arm position = [{0},{1},{2},{3},{4},{5}]".format(position[0], position[1], position[2], position[3], position[4], position[5]))
#
#     rospy.sleep(rospy.Duration(1.0))
#
#     resp.solution.joint_state.header.stamp = rospy.Time.now()
#     pub_joint_states.publish(resp.solution.joint_state)
#
#     rospy.sleep(rospy.Duration(1.0))
