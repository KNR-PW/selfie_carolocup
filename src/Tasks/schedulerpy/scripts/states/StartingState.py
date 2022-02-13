#!/usr/bin/env python3

from .SelfieState import *
import topic_tools
import custom_msgs.msg
from utils import ActionCallbackPack
from enum import IntEnum
from custom_msgs.msg import startingResult
from smach.user_data import UserData
from smach.state import State


class ModeEnum(IntEnum):
    NOT_INITIALIZED = -1
    PARKING = 0
    INTERSECTION = 1


class StartingState(SelfieState):

    def __init__(self, state_outcomes):
        SelfieState.__init__(self,
                             state_outcomes,
                             output_keys=['state_output'])

        self._mux_drive_select = rospy.ServiceProxy("drive_multiplexer/select",
                                                    topic_tools.srv.MuxSelect)

        self.set_initial_goal()
        self._driving_mode: int = ModeEnum.NOT_INITIALIZED

    def set_initial_goal(self):
        initial_goal = custom_msgs.msg.startingGoal()
        initial_goal.distance = 1.0
        self._current_goal = initial_goal

    def prepare_action(self, user_data: UserData):
        self._mux_drive_select.call("drive/starting_procedure")
        rospy.loginfo("Prepare starting - MuxSelect drive/starting_procedure")

    def service_state_result(self, userdata: UserData):
        userdata.state_output = self._driving_mode

    def redirect_state(self) -> str:
        return self.current_outcomes[0]


class StartingStateCallbackPack(ActionCallbackPack.ActionCallbackPack):

    def doneCallback(self, state, result: startingResult):
        self.state._driving_mode = result.drive_mode
        rospy.loginfo("Starting procedure finished")

    def activeCallback(self):
        rospy.loginfo("Starting procedure")


class StartingStateBuilder(SelfieStateBuilder):

    def __init__(self) -> None:
        SelfieStateBuilder.__init__(self, 'task/starting_procedure',
                                    ['StartingOutcome'],
                                    custom_msgs.msg.startingAction)
        self._callback_pack = StartingStateCallbackPack()
        self.reset()

    def _reset(self) -> None:
        self._state_to_build = StartingState(self.outcomes)
