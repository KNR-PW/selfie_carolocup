#!/usr/bin/env python3

import custom_msgs.msg
import std_srvs.srv
import topic_tools
import topic_tools.srv
from utils import ActionCallbackPack
from threading import Lock, Condition

from .SelfieState import *


class FreeDriveState(SelfieState):

    def __init__(self, state_outcomes: List[str]):
        SelfieState.__init__(self, state_outcomes)

        self.mux_drive_select_ = rospy.ServiceProxy("drive_multiplexer/select",
                                                    topic_tools.srv.MuxSelect)

        self._vision_reset = rospy.ServiceProxy("resetVision",
                                                std_srvs.srv.Empty)
        self._reset_lane_controller = rospy.ServiceProxy(
            "resetLaneControl", std_srvs.srv.Empty)
        self._mux_drive_select = rospy.ServiceProxy("drive_multiplexer/select",
                                                    topic_tools.srv.MuxSelect)
        self._avoiding_obst_set_passive = rospy.ServiceProxy(
            "avoiding_obst_set_passive", std_srvs.srv.Empty)
        self._avoiding_obst_set_active = rospy.ServiceProxy(
            "avoiding_obst_set_active", std_srvs.srv.Empty)

        self._is_manual_mode = False

        self._lock = Lock()
        self._control_mode_guardian = Condition(self._lock)

        self._current_competition: int = CompetitionID.NONE

    def set_manual_mode(self):
        self._is_manual_mode = True

    def set_auto_mode(self):
        with self._control_mode_guardian:
            self._control_mode_guardian.notify()

    def wait_for_auto_mode(self):
        with self._control_mode_guardian:
            self._control_mode_guardian.wait()

    def _try_run_active_obstacle_detector(self):
        try:
            print('ACTIVE OBSTACLE DETECTOR')
            empty_msg = std_srvs.srv.EmptyRequest()
            self._avoiding_obst_set_active.call(empty_msg)
        except rospy.service.ServiceException as error:
            print('CALLING ACTIVE OBSTACLE DETECTOR FAILED')
            rospy.loginfo(error)

    def _reset_hardware(self):
        self._vision_reset.call(std_srvs.srv.EmptyRequest())
        self._reset_lane_controller.call(std_srvs.srv.EmptyRequest())

    def _take_control_over_multiplexer(self):
        self._mux_drive_select.call("drive/lane_control")

    def enable_passive_obstacle_detector(self):
        empty_msg = std_srvs.srv.EmptyRequest()
        self._avoiding_obst_set_passive.call(empty_msg)

    def setup_goal(self, user_data: UserData) -> None:
        self._current_competition = user_data.competition
        self.current_goal = custom_msgs.msg.drivingGoal()
        # TODO: remove bool type from driving.action
        self.current_goal.mode = bool(user_data.competition)

    def prepare_action(self, user_data: UserData) -> None:
        rospy.loginfo("Preparing free drive")

        self._reset_hardware()
        self._take_control_over_multiplexer()

        self._try_run_active_obstacle_detector()

    def execute(self, user_data: UserData) -> str:
        if self._is_manual_mode:
            self.wait_for_auto_mode()

        return SelfieState.execute(self, user_data)

    def redirect_state(self) -> str:
        return self.current_outcomes[self._current_competition]


class FreeDriveStateCallbackPack(ActionCallbackPack.ActionCallbackPack):

    def doneCallback(self, state, result):
        self._state.enable_passive_obstacle_detector()

    def activeCallback(self):
        rospy.loginfo("Drive action server active")


class FreeDriveStateBuilder(SelfieStateBuilder):

    def __init__(self) -> None:
        SelfieStateBuilder.__init__(self, 'task/free_drive',
                                    custom_msgs.msg.drivingAction,
                                    ['GoToParkingState', "GoToIntersectionState"])
        self._callback_pack = FreeDriveStateCallbackPack()
        self.reset()

    def _reset(self) -> None:
        self._state_to_build = FreeDriveState(self.outcomes)

        # TODO: change it later

        goal_to_set = custom_msgs.msg.drivingGoal()
        goal_to_set.mode = True

        self._state_to_build.current_goal = goal_to_set
