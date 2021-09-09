import actionlib
import rospy

from enum import Enum, auto

from std_msgs.msg import Header
from custom_msgs.msg import crosswalkGoal, crosswalkAction, Box2DArray, Box2D
from geometry_msgs.msg import Point

from common_py import filter_boxes, create_square_box


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
        self._sub_obstacles = None  # to be activated later

        # load parameters
        self.visualization = rospy.get_param("visualization", default=True)
        self.roi_min_x = rospy.get_param("roi_min_x", default=-0.1)
        self.roi_max_x = rospy.get_param("roi_max_x", default=0.5)
        self.roi_min_y = rospy.get_param("roi_min_y", default=-0.5)
        self.roi_max_y = rospy.get_param("roi_max_y", default=0.5)

        # Others
        # Region of interest
        self.roi_box = create_square_box(self.roi_min_x, self.roi_min_y,
                                         self.roi_max_x, self.roi_max_y)

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
