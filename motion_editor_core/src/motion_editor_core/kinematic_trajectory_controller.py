import roslib
roslib.load_manifest('motion_editor_core')

import rospy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .joint_configuration import appendixes

topic_prefix = '/thor_mang'

class KinematicTrajectoryController(object):

    def __init__(self, appendix):
        self._appendix = appendix
        self._running = False
        self._joint_states = JointState()
        self._joint_states.name = self._appendix['joint_names']
        self._joint_states.position = [0.0] * len(self._appendix['joint_names'])
        self._current_joint_states = JointState()
        self._state_subscriber = rospy.Subscriber('%s/joint_states' % (topic_prefix), JointState, self._state_callback)
        self._state_publisher = rospy.Publisher('%s/%s/joint_states' % (topic_prefix, self._appendix['controller_topic']), JointState)
        self._trajectory_subscriber = rospy.Subscriber('%s/%s/trajectory' % (topic_prefix, self._appendix['controller_topic']), JointTrajectory, self._trajectory_callback)


    def _execute_trajectorie(self, trajectory):
        if self._running:
            return
        self._running = True

        joint_ids = [self._current_joint_states.name.index(joint_name) for joint_name in self._appendix['joint_names']]
        position_previous = JointTrajectoryPoint()
        position_previous.positions = [self._current_joint_states.position[joint_id] for joint_id in joint_ids]
        #print 'starting from: ' + ('%6.2f ' * len(joint_ids)) % tuple(position_previous.positions)
        position_previous.time_from_start = rospy.Duration(0.0)
        position_index = 0
        start_time = rospy.get_rostime()
        while position_index < len(trajectory.points):
            now = rospy.get_rostime() - start_time
            #print '           to: ' + ('%6.2f ' * len(joint_ids)) % tuple(trajectory.points[position_index].positions)
            while now <= trajectory.points[position_index].time_from_start:
                delta_time = now - position_previous.time_from_start
                position_duration = trajectory.points[position_index].time_from_start - position_previous.time_from_start
                if position_duration.to_sec() == 0.0:
                    progress = 1.0
                else:
                    progress = delta_time.to_sec() / position_duration.to_sec()
                for joint_index, goal_position in enumerate(trajectory.points[position_index].positions):
                    joint_delta = progress * (goal_position - position_previous.positions[joint_index])
                    joint_position = position_previous.positions[joint_index] + joint_delta
                    self._joint_states.position[joint_index] = joint_position
                self._state_publisher.publish(self._joint_states)
                now = rospy.get_rostime() - start_time
                rospy.sleep(0.01)
            position_previous = trajectory.points[position_index]
            position_index += 1

        self._running = False


    def _trajectory_callback(self, trajectory):
        self._execute_trajectorie(trajectory)


    def _state_callback(self, joint_states):
        self._current_joint_states = joint_states


    def shutdown(self):
        self._state_publisher.unregister()
        self._state_subscriber.unregister()
        self._trajectory_subscriber.unregister()


def main():
    rospy.init_node('kinematic_trajectory_controller')
    trajectory_controllers = {}
    for appendix in appendixes:
        trajectory_controllers[appendix['name']] = KinematicTrajectoryController(appendix)
    rospy.spin()
    #for appendix in appendixes:
    #    trajectory_controllers[appendix['name']].shutdown()

