#!/usr/bin/env python3

import custom_msgs.msg
import topic_tools
from custom_msgs.msg import startingResult
from smach.user_data import UserData
from utils import ActionCallbackPack

from .SelfieState import *
from .SelfieState import CompetitionID


class StartingState(SelfieState):

    def __init__(self, state_outcomes):
        SelfieState.__init__(self, state_outcomes)

        self._mux_drive_select = rospy.ServiceProxy("drive_multiplexer/select",
                                                    topic_tools.srv.MuxSelect)

        self._competition: int = CompetitionID.NONE

    def setup_goal(self, user_data: UserData) -> None:
        initial_goal = custom_msgs.msg.startingGoal()
        initial_goal.distance = 1.0
        self._current_goal = initial_goal

    def prepare_action(self, user_data: UserData):
        self._mux_drive_select.call("drive/starting_procedure")
        rospy.loginfo("Prepare starting - MuxSelect drive/starting_procedure")

    def service_state_result(self, userdata: UserData):
        userdata.competition = self._competition

    def redirect_state(self) -> str:
        return self.current_outcomes[0]


class StartingStateCallbackPack(ActionCallbackPack.ActionCallbackPack):

    def doneCallback(self, state, result: startingResult):
        self._state._competition = result.drive_mode
        rospy.loginfo("Starting procedure finished")

    def activeCallback(self):
        rospy.loginfo("Starting procedure")


class StartingStateBuilder(SelfieStateBuilder):

    def __init__(self) -> None:
        SelfieStateBuilder.__init__(self, 'task/starting_procedure',
                                    custom_msgs.msg.startingAction)
        self._callback_pack = StartingStateCallbackPack()
        self.reset()

    def _reset(self) -> None:
        self._state_to_build = StartingState(self.outcomes)
