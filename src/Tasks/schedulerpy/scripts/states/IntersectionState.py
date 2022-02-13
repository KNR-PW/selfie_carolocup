#!/usr/bin/env python3

from .SelfieState import *
import topic_tools
import custom_msgs.msg
import std_srvs.srv
import topic_tools.srv

from utils import ActionCallbackPack


class IntersectionState(SelfieState):

    def __init__(self, state_outcomes):
        SelfieState.__init__(self, state_outcomes)

        self._current_goal = custom_msgs.msg.intersectionGoal()

        self._reset_lane_controller = rospy.ServiceProxy(
            "resetLaneControl", std_srvs.srv.Empty)

        self._avoiding_obst_set_passive = rospy.ServiceProxy(
            "avoiding_obst_set_passive", std_srvs.srv.Empty)

    def prepare_action(self, user_data):
        self._reset_lane_controller.call(std_srvs.srv.EmptyRequest())
        self._avoiding_obst_set_passive.call(std_srvs.srv.EmptyRequest())

        rospy.loginfo(
            "Prepare intersection - call reset Lane control and avoiding obst passive"
        )

    def redirect_state(self):
        return self.current_outcomes[0]


class IntersectionStateCallbackPack(ActionCallbackPack.ActionCallbackPack):

    def doneCallback(self, state, result):
        rospy.loginfo("Finished intersection state")

    def activeCallback(self):
        rospy.loginfo("Intersection action active")


class IntersectionStateBuilder(SelfieStateBuilder):

    def __init__(self) -> None:
        SelfieStateBuilder.__init__(self, 'task/intersection',
                                    ['IntersectionOutcome'],
                                    custom_msgs.msg.intersectionAction)
        self._callback_pack = IntersectionStateCallbackPack()
        self.reset()

    def _reset(self) -> None:
        self._state_to_build = IntersectionState(self.outcomes)
