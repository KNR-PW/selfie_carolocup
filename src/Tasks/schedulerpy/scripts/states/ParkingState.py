#!/usr/bin/env python3

from SelfieState import *
import topic_tools
import custom_msgs.msg
import std_srvs.srv
from topic_tools.srv import MuxSelect
from .SelfieState import CompetitionID

from utils import ActionCallbackPack


class ParkingState(SelfieState):

    def __init__(self, state_outcomes):
        SelfieState.__init__(self, state_outcomes)

        self.mux_drive_select_ = rospy.ServiceProxy("drive_multiplexer/select",
                                                    topic_tools.srv.MuxSelect)

    def redirect_state(self) -> str:
        return self.current_outcomes[0]

    def setup_goal(self, user_data: UserData) -> None:
        # TODO: make sure that i can read parking_spot from in_args
        self.current_goal = user_data.in_args.parking_spot

    def prepare_action(self, user_data: UserData) -> None:
        mux_msg = topic_tools.srv.MuxSelectRequest()
        mux_msg.topic = "drive/park"
        self.mux_drive_select_.call(mux_msg)


class ParkingStateCallbackPack(ActionCallbackPack.ActionCallbackPack):

    def doneCallback(self, state, result):
        rospy.loginfo("Finished parking state")

    def activeCallback(self):
        rospy.loginfo("Parking action active")


class ParkingStateBuilder(SelfieStateBuilder):

    def __init__(self) -> None:
        SelfieStateBuilder.__init__(self, 'task/park', custom_msgs.msg.parkAction)
        self.callbackPack = ParkingStateCallbackPack()
        self.reset()

    def _reset(self) -> None:
        self._state_to_build = ParkingState(self.outcomes)
