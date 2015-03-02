import roslib
roslib.load_manifest('flor_motion')

import rospy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .joint_configuration import appendixes

topic_prefix = '/thor_mang'

class MotionPublisher(object):
    _precision = 4

    def __init__(self):
        self._joint_states = {}
        self._state_subscribers = {}
        self._subscriber_prefix = None
        self.set_subscriber_prefix(topic_prefix)
        self._publisher_prefix = topic_prefix

        self._trajectory_publisher = {}
        for appendix in appendixes:
            self._trajectory_publisher[appendix['name']] = TrajectoryPublisher(appendix)


    def set_subscriber_prefix(self, prefix):
        print 'Setting subscriber prefix to:', prefix
        if self._subscriber_prefix == prefix:
            return
        self._subscriber_prefix = prefix
        if prefix not in self._state_subscribers:
            self._joint_states[prefix] = JointState()
            joint_state_topic = '%s/joint_states' % prefix
            print 'Subscribing to topic:', joint_state_topic
            self._state_subscribers[prefix] = rospy.Subscriber(joint_state_topic, JointState, self._state_callback, prefix)


    def set_publisher_prefix(self, prefix):
        print 'Setting publisher prefix to:', prefix
        self._publisher_prefix = prefix
        for publisher in self._trajectory_publisher.values():
            publisher.set_publisher_prefix(self._publisher_prefix)


    def move_to_position(self, appendix_name, target_position, duration=1.0):
        target_pos = {
            'name': 'target',
            'starttime': 0.0,
            'duration': duration,
            'positions': target_position,
        }
        motion = {appendix_name: [target_pos]}
        self.publish_motion(motion)


    def publish_motion(self, motion, time_factor=1.0):
        for appendix in appendixes:
            motion_list = motion.get(appendix['name'], [])
            current_positions = self.get_current_positions(appendix)
            if len(motion_list) > 0 and current_positions is not None:
                self._trajectory_publisher[appendix['name']].publish_trajectory(current_positions, motion_list, time_factor=time_factor)


    def get_current_positions(self, appendix):
        try:
            joint_ids = [self._joint_states[self._subscriber_prefix].name.index(joint_name) for joint_name in appendix['joint_names']]
        except ValueError as e:
            print 'Error: Some joint was not found in received joint state:\n%s' % (e,)
            return None
        current_positions = [round(self._joint_states[self._subscriber_prefix].position[joint_id], self._precision) for joint_id in joint_ids]
        return current_positions


    def _state_callback(self, joint_states, prefix):
        self._joint_states[prefix] = joint_states


    def shutdown(self):
        for appendix in appendixes:
            self._trajectory_publisher[appendix['name']].shutdown()
        self._state_subscriber.unregister()


class TrajectoryPublisher(object):

    def __init__(self, appendix):
        self._appendix = appendix
        self._trajectory = JointTrajectory()
        self._publisher_prefix = None
        self._publishers = {}
        self.set_publisher_prefix(topic_prefix)


    def set_publisher_prefix(self, prefix):
        if self._publisher_prefix == prefix:
            return
        self._publisher_prefix = prefix
        if prefix not in self._publishers:
            self._publishers[prefix] = rospy.Publisher('%s/%s/command' % (prefix, self._appendix['controller_topic']), JointTrajectory, queue_size=1000)
            print 'Publishing to topic:', '%s/%s/command' % (prefix, self._appendix['controller_topic'])


    def _add_point(self, time, positions):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = len(point.positions) * [0.0]
        point.time_from_start = rospy.Duration(time)
        self._trajectory.points.append(point)


    def publish_trajectory(self, current_positions, motion_list, time_factor = 1.0):
        self._trajectory.header.stamp = rospy.Time.now()
        self._trajectory.header.seq += 1
        self._trajectory.joint_names = self._appendix['joint_names']
        self._trajectory.points = []
        start_pose = {
            'name': 'current',
            'starttime': 0.0,
            'duration': 0.0,
            'positions': current_positions,
        }
        motion_list = [start_pose] + motion_list
        last_motion = None
        for motion in motion_list:
            if last_motion is not None:
                endtime = last_motion['starttime'] * time_factor + last_motion['duration'] * time_factor
                # if last motion ended before next one starts, hold old positions
                if motion['starttime'] * time_factor - endtime > 0.01:
                    self._add_point(motion['starttime'] * time_factor, last_motion['positions'])
            # move to next positions
            time = motion['starttime'] * time_factor + motion['duration'] * time_factor
            self._add_point(time, motion['positions'])
            last_motion = motion

        self._publishers[self._publisher_prefix].publish(self._trajectory)

    def shutdown(self):
        for pub in self._publishers.values():
            pub.unregister()
