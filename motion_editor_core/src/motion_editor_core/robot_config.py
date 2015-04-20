#!/usr/bin/python
__author__ = 'Martin Oehler'

import roslib
roslib.load_manifest('motion_editor_core')

from xml.etree import ElementTree
from rospkg import RosPack


class RobotConfigLoader():
    def __init__(self):
        self.robot_config = None
        self.targets = []

    def load_xml_by_path(self, path):
        tree = ElementTree.parse(path)
        root = tree.getroot()
        self.parse_xml(root)

    def load_xml_by_name(self, name):
        ros_pack = RosPack()
        path = ros_pack.get_path('motion_editor_core') + '/config/' + name
        self.load_xml_by_path(path)

    def parse_xml(self, root):
        if root.tag != 'config':
            print 'Motion editor config should start with a "config" node.'
            return None
        for robot_node in root:
            if robot_node.tag == 'robot':
                self.robot_config = RobotConfig()
                for group_node in robot_node:
                    if group_node.tag != 'group':
                        print 'Expected "group" tag but found', group_node.tag
                        return
                    try:
                        group = JointGroup(group_node.attrib['name'], group_node.attrib['type'])
                    except KeyError as e:
                        print 'Group node has incomplete attributes:', e
                        return
                    self.robot_config.add_group(group)
                    for joint_node in group_node:
                        if joint_node.tag != 'joint':
                            print 'Expected "joint" tag but found', joint_node.tag
                            return
                        try:
                            joint = Joint(joint_node.attrib['name'])
                        except KeyError as e:
                            print 'Joint node has incomplete attributes:', e
                            return
                        joint.mirrored = group_node.attrib.get('mirrored', "false") == 'true' or \
                                         joint_node.attrib.get('mirrored', "false") == 'true'
                        group.add_joint(joint)
            elif robot_node.tag == 'target':
                try:
                    target = MotionTarget(robot_node.attrib['name'], robot_node.attrib['pub'], robot_node.attrib['sub'])
                except KeyError as e:
                    print "MotionTarget node has incomplete attributes:", e
                    return
                self.targets.append(target)
            else:
                print 'Expected "robot" or "target" tag. Found:', robot_node.tag
                return


class MotionTarget():
    def __init__(self, name, pub, sub):
        self.name = name
        self.pub = pub
        self.sub = sub

    def __str__(self):
        return 'MotionTarget(name=%s, pub=%s, sub=%s)' % (self.name, self.pub, self.sub)


class RobotConfig():
    def __init__(self):
        self.groups = {}

    def add_group(self, group):
        self.groups[group.name] = group

    def get_types(self):
        return set([group.type for group in self.groups.itervalues()])

    def __str__(self):
        neat_string = 'Semantic description: \n'
        for group in self.groups.itervalues():
            neat_string += ' -- ' + group.__str__() + '\n'
        return neat_string


class JointGroup():
    def __init__(self, name, group_type):
        self.name = name
        self.group_type = group_type
        self.joints = {}

    def add_joint(self, joint):
        self.joints[joint.name] = joint

    def adapt_to_side(self, positions):
        return [-position if joint.mirrored else position for joint, position in zip(self.joints, positions)]

    def __str__(self):
        neat_string = 'Group name: ' + self.name + ' Type: ' + self.group_type + '\n'
        for joint in self.joints.itervalues():
            neat_string += '   -- ' + joint.__str__() + '\n'
        return neat_string


class Joint():
    def __init__(self, name, mirrored=False):
        self.name = name
        self.mirrored = mirrored

    def __str__(self):
        return self.name + (' [mirrored]' if self.mirrored else '')


if __name__ == '__main__':
    loader = RobotConfigLoader()
    loader.load_xml_by_name('motion_editor_thor_config.xml')
    _robot_config = loader.robot_config
    _group = _robot_config.groups['r_arm']
    print _robot_config
    for _target in loader.targets:
        print _target

