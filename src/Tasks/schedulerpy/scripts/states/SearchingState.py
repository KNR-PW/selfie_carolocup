from threading import Lock, Condition
from typing import Optional

import custom_msgs.msg
from std_msgs.msg import Float32
from utils import ActionCallbackPack

from SelfieState import *


class SearchingState(SelfieState):

    def __init__(self, state_outcomes):
        SelfieState.__init__(self, state_outcomes)
        self._lock = Lock()
        self._result_set_guardian = Condition(self._lock)
        self._parking_spot = custom_msgs.msg.Box2D()
        self._is_searching_ok = True

    def _reset_searching_status_flag(self):
        self._is_searching_ok = True

    def redirect_state(self) -> str:
        found_place_to_park = self._is_searching_ok
        if found_place_to_park:
            return self.current_outcomes[1]
        return self.current_outcomes[0]

    def setup_goal(self, user_data: UserData) -> None:
        new_goal = custom_msgs.msg.searchGoal()
        new_goal.min_spot_lenght = 0.6
        self.current_goal = new_goal

    def prepare_action(self, user_data: UserData) -> None:
        pass

    def _wait_for_result_set(self):
        with self._result_set_guardian:
            self._result_set_guardian.wait()

    def notify_result_guardian(self):
        with self._result_set_guardian:
            self._result_set_guardian.notify()

    def service_state_result(self, userdata: UserData):
        self.wait_for_result()
        spot = Float32()
        spot.data = self._parking_spot
        userdata.out_args.parking_spot = spot
        rospy.loginfo("Finishing action!")


class SearchingStateCallbackPack(ActionCallbackPack.ActionCallbackPack):

    def doneCallback(self, state: actionlib.SimpleGoalState, result: custom_msgs.msg.searchResult) -> None:
        self._state._parking_spot = result.parking_spot
        if state != actionlib.GoalStatus.SUCCEEDED:
            self._state._is_searching_ok = False
        self._state.notify_result_guardian()
        rospy.loginfo("Finished searching state")

    def activeCallback(self) -> None:
        rospy.loginfo("Searching action active")


class SearchingStateBuilder(SelfieStateBuilder):

    def __init__(self) -> None:
        SelfieStateBuilder.__init__(self, 'task/parking_spot_detector', custom_msgs.msg.searchAction,
                                    ['GoToParkingState'])
        self.callbackPack = SearchingStateCallbackPack()
        self.reset()

    def _reset(self) -> None:
        self.state = SearchingState(self.outcomes)
