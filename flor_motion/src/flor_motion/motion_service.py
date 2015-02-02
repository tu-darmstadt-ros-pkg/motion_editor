import roslib
roslib.load_manifest('flor_motion')
import rospy
from flor_control_msgs.srv import FlorExecuteMotion, FlorExecuteMotionResponse
from .motion_publisher import MotionPublisher
from .motion_data import MotionData
from flor_ocs_msgs.msg import OCSRobotStatus
from flor_control_msgs.msg import FlorExecuteMotionCommand

class MotionService(object):

    def __init__(self):
        self._motion_publisher = MotionPublisher()
        self._motion_publisher.set_subscriber_prefix('/atlas')
        self._motion_publisher.set_publisher_prefix('/flor')
        self._motion_data = MotionData()
        self.motion_command_sub       = rospy.Subscriber('/flor/motion_service/motion_command', FlorExecuteMotionCommand, self.motionCommandRequestCallbackFnc)
        self.motion_server_status_pub = rospy.Publisher( '/flor/motion_service/status',OCSRobotStatus)
        self.motion_server_status     = OCSRobotStatus()

    def _execute_motion(self, request):
        response = FlorExecuteMotionResponse()

        # check if a motion with this name exists
        if request.motion_name not in self._motion_data:
            print 'MotionService: unknown motion name: "%s"' % request.motion_name

            # Send status code to OCS
            #MOTION_SERVICE_UKNOWN_MOTION                 = 60,
            self.motion_server_status.stamp = rospy.Time.now()
            self.motion_server_status.code = 60 + (3<<6) # set status code and severity level
            self.motion_server_status_pub.publish(self.motion_server_status)

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

        # Send status code to OCS
        #MOTION_SERVICE_INITIATE_MOTION              = 61,
        #self.motion_server_status.stamp = rospy.Time.now()
        #self.motion_server_status.code = 61 # set status code and severity level
        #self.motion_server_status_pub.publish(self.motion_server_status)

        # execute motion
        print 'MotionService: executing motion: "%s" (time factor: %.3f, duration %.2fs)' % (request.motion_name, request.time_factor, motion_duration)
        self._motion_publisher.publish_motion(self._motion_data[request.motion_name], request.time_factor)

        # Send status code to OCS
        #MOTION_SERVICE_MOTION_COMPLETE              = 63,
        #self.motion_server_status.stamp = rospy.Time.now()
        #self.motion_server_status.code = 63 # set status code and severity level
        #self.motion_server_status_pub.publish(self.motion_server_status)

        # reply with ok and the finish_time of this motion
        response.ok = True
        response.finish_time.data = rospy.Time.now() + rospy.Duration.from_sec(motion_duration)
        return response

    def start_server(self):
        rospy.init_node('flor_motion_service')
        rospy.Service('flor/motion_service/execute_motion', FlorExecuteMotion, self._execute_motion)
        print 'MotionService: waiting for calls...'
        rospy.spin()

    def motionCommandRequestCallbackFnc(self, command):
        print "Initiate motion command via topic ..."
        request = FlorExecuteMotion()
        request.motion_name = command.motion_name
        request.time_factor = command.time_factor
        self._execute_motion(request)
        print "Motion command complete"

def main():
    motion_service = MotionService()
    motion_service.start_server()
