import roslib
roslib.load_manifest('flor_motion')
import rospy
from flor_motion.srv import ExecuteMotion, ExecuteMotionResponse
from .motion_publisher import MotionPublisher
from .motion_data import MotionData
from flor_motion.msg import ExecuteMotionCommand

class MotionService(object):

    def __init__(self):
        self._motion_publisher = MotionPublisher()
        self._motion_publisher.set_subscriber_prefix('/thor_mang')
        self._motion_publisher.set_publisher_prefix('/thor_mang')
        self._motion_data = MotionData()
        self.motion_command_sub = rospy.Subscriber('/thor_mang/motion_service/motion_command', ExecuteMotionCommand, self.motionCommandRequestCallbackFnc)

    def _execute_motion(self, request):
        response = ExecuteMotionResponse()

        # check if a motion with this name exists
        if request.motion_name not in self._motion_data:
            print 'MotionService: unknown motion name: "%s"' % request.motion_name

            response.ok = False
            response.finish_time.data = rospy.Time.now()
            return response

        # check if a valid time_factor is set, otherwise default to 1.0
        if request.time_factor <= 0.0:
            request.time_factor = 1.0

        # find the duration of the requested motion
        motion_duration = 0.0
        for poses in self._motion_data[request.motion_name].values():
            if len(poses) > 0:
                endtime = poses[-1]['starttime'] + poses[-1]['duration']
                motion_duration = max(motion_duration, endtime)
        motion_duration *= request.time_factor

        # execute motion
        print 'MotionService: Executing motion: "%s" (time factor: %.3f, duration %.2fs)' % (request.motion_name, request.time_factor, motion_duration)
        self._motion_publisher.publish_motion(self._motion_data[request.motion_name], request.time_factor)

        # reply with ok and the finish_time of this motion
        response.ok = True
        response.finish_time.data = rospy.Time.now() + rospy.Duration.from_sec(motion_duration)
        return response

    def start_server(self):
        rospy.init_node('flor_motion_service')
        rospy.Service('thor_mang/motion_service/execute_motion', ExecuteMotion, self._execute_motion)
        print 'MotionService: Waiting for calls...'
        rospy.spin()

    def motionCommandRequestCallbackFnc(self, command):
        print "Initiate motion command via topic ..."
        request = ExecuteMotion()
        request.motion_name = command.motion_name
        request.time_factor = command.time_factor
        self._execute_motion(request)
        print "Motion command complete"

def main():
    motion_service = MotionService()
    motion_service.start_server()
