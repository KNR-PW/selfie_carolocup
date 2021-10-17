import actionlib
import rospy

from enum import Enum, auto

from std_msgs.msg import Header
from custom_msgs.msg import crosswalkGoal, crosswalkAction, Box2DArray, Box2D, Motion
from geometry_msgs.msg import Point

from common_py import filter_boxes, create_square_box


class CrosswalkManager:
    ACTION_NAME = "task/crosswalk"

    class ActionState(Enum):
        DISABLED = auto()
        IDLE = auto()
        APPROACHING_TO_EMPTY_CROSSWALK = auto()
        APPROACHING_TO_NOT_EMPTY_CROSSWALK = auto()
        WAITING_FOR_PEDESTRIANS = auto()

    def __init__(self):
        self.state = self.ActionState.DISABLED
        self.distance_current = 0
        # init action
        self._action_service = actionlib.SimpleActionServer(
            self.ACTION_NAME,
            crosswalkAction,
            execute_cb=self._action_goal_received,
            auto_start=False)
        # init subscribers
        self._sub_obstacles = None  # to be activated later
        self._sub_distance = rospy.Subscriber(
            "/road_lines", Motion, callback=self._crosswalk_distance_cb)

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
        self._update_state(self.ActionState.IDLE)

    def _action_goal_received(self, goal: crosswalkGoal):
        rospy.loginfo("Crosswalk action started")
        self._update_state(self.ActionState.APPROACHING_TO_EMPTY_CROSSWALK)
        self._start_listening()

    def _obstacles_cb(self, boxes: Box2DArray):
        filtered_boxes = filter_boxes(
            self.roi_box,
            boxes.boxes)  # maybe add distance to crosswalk to x of roi box

        # NOTE it would be good to somehow remove box representing sign from obstacles list
        if filtered_boxes:
            rospy.logdebug(
                f"Detected {len(filtered_boxes)} boxes in area of interest")
            if (self.state == self.ActionState.APPROACHING_TO_EMPTY_CROSSWALK):
                self._update_state(
                    self.ActionState.APPROACHING_TO_NOT_EMPTY_CROSSWALK)
        else:
            if (self.state == self.ActionState.WAITING_FOR_PEDESTRIANS):
                pass  # TODO end task
            elif (self.state ==
                  self.ActionState.APPROACHING_TO_NOT_EMPTY_CROSSWALK):
                self._update_state(
                    self.ActionState.APPROACHING_TO_EMPTY_CROSSWALK)

    def _crosswalk_distance_cb(self, motion: Motion):
        self.distance_current = motion.distance
        # TODO distance from motion or just distance to crosswalk

    def _start_listening(self):
        rospy.logdebug("Subscribing to used topics")
        self._sub_obstacles = rospy.Subscriber("/obstacles",
                                               Box2DArray,
                                               callback=self._obstacles_cb)

    def _stop_listening(self):
        rospy.logdebug("Unsubscribing from used topics")
        self._sub_obstacles.unregister()

    def _update_state(self, new_state: ActionState):
        self.state = new_state
        rospy.logdebug(f'Changed crosswalk task state to: {self.state.name}')
