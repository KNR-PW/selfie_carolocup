import actionlib
import rospy

from enum import Enum, auto

from std_msgs.msg import String
from custom_msgs.msg import crosswalkGoal, crosswalkAction, Box2DArray


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(5)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


class CrosswalkManager:
    ACTION_NAME = "task/crosswalk"

    class ActionState(Enum):
        DISABLED = auto()
        IDLE = auto()
        APPROACHING_TO_CROSSWALK = auto()
        WAITING_FOR_PEDESTRIANS = auto()

    def __init__(self):
        self.state = self.ActionState.DISABLED
        # init action
        self._action_service = actionlib.SimpleActionServer(
            self.ACTION_NAME,
            crosswalkAction,
            execute_cb=self._action_goal_received,
            auto_start=False)
        # init subscribers
        self._sub_obstacles = rospy.Subscriber()  # to be activated later

    def start(self):
        rospy.loginfo("Starting crosswalk action server")
        self._action_service.start()
        self.state = self.ActionState.IDLE

    def _action_goal_received(self, goal: crosswalkGoal):
        rospy.loginfo("Crosswalk action started")
        self.state = self.ActionState.APPROACHING_TO_CROSSWALK
        self._start_listening()

    def _obstacles_cb(self, boxes: Box2DArray):
        pass

    def _start_listening(self):
        rospy.logdebug("Subscribing to used topics")
        self._sub_obstacles = rospy.Subscriber("/obstacles",
                                               Box2DArray,
                                               callback=self._obstacles_cb)

    def _stop_listening(self):
        self._sub_obstacles.unregister()
