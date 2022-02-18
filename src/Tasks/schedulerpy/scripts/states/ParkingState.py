#!/usr/bin/env python3

from SelfieState import *
import topic_tools
import custom_msgs.msg
import std_srvs.srv
import topic_tools.srv

from utils import ActionCallbackPack


class ParkingState(SelfieState):

    def __init__(self, state_outcomes):
        SelfieState.__init__(self, state_outcomes)

        self.muxDriveSelect_ = rospy.ServiceProxy("drive_multiplexer/select",
                                                  topic_tools.srv.MuxSelect)

    def prepare_action(self):
        self.muxDriveSelect_.call("drive/park")


class ParkingStateCallbackPack(ActionCallbackPack.ActionCallbackPack):

    def doneCallback(self, state, result):
        rospy.loginfo("Finished parking state")

    def activeCallback(self):
        rospy.loginfo("Parking action active")


class ParkingStateBuilder(SelfieStateBuilder):

    def __init__(self) -> None:
        self.outcomes = ["ParkingOutcome"]
        self.actionName = "task/park"
        self.actionType = custom_msgs.msg.parkAction
        self.reset()
        self.callbackPack = ParkingStateCallbackPack()

    def __reset(self) -> None:
        self.state = ParkingState(self.outcomes)

    def product(self) -> ParkingState:
        state = self.state
        self.reset()
        return state
