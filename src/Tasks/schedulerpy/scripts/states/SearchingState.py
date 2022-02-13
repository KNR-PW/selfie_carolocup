#!/usr/bin/env python3

from SelfieState import *
import topic_tools
import custom_msgs.msg
import std_srvs.srv
import topic_tools.srv

from utils import ActionCallbackPack


class SearchingState(SelfieState):

    def __init__(self, state_outcomes):
        SelfieState.__init__(self, state_outcomes)

    def prepare_action(self):
        pass


class SearchingStateCallbackPack(ActionCallbackPack.ActionCallbackPack):

    def doneCallback(self, state, result):
        rospy.loginfo("Finished searching state")

    def activeCallback(self):
        rospy.loginfo("Searching action active")


class IntersectionStateBuilder(SelfieStateBuilder):

    def __init__(self) -> None:
        self.outcomes = ["SearchingOutcome"]
        self.actionName = "task/parking_spot_detector"
        self.actionType = custom_msgs.msg.searchAction
        self.reset()
        self.callbackPack = SearchingStateCallbackPack()

    def __reset(self) -> None:
        self.state = SearchingState(self.outcomes)

    def product(self) -> SearchingState:
        state = self.state
        self.reset()
        return state
