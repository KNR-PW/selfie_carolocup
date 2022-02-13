#!/usr/bin/env python3
import imp
import rospy

from .SelfieState import *
from .StartingState import ModeEnum
import topic_tools
import custom_msgs.msg
import std_srvs.srv
import topic_tools.srv
from typing import List
from utils import ActionCallbackPack

from enum import Enum


class FreeDriveState(SelfieState):

    def __init__(self, state_outcomes: List[str]):
        SelfieState.__init__(self, state_outcomes, input_keys=['state_input'])

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
        self._drive_mode: int = ModeEnum.NOT_INITIALIZED

    def try_run_active_obstacle_detector(self):
        try:
            print('ACTIVE OBSTACLE DETECTOR')
            empty_msg = std_srvs.srv.EmptyRequest()
            self._avoiding_obst_set_active.call(empty_msg)
        except rospy.service.ServiceException as error:
            rospy.loginfo(error)

    def reset_hardware(self):
        self._vision_reset.call(std_srvs.srv.EmptyRequest())
        self._reset_lane_controller.call(std_srvs.srv.EmptyRequest())

    def take_control_over_multiplexer(self):
        self._mux_drive_select.call("drive/lane_control")

    def extract_drive_mode_from_result(self, result):
        self._state._drive_mode = result.drive_mode

    def prepare_action(self, user_data):
        self.reset_hardware()
        self.take_control_over_multiplexer()

        print(user_data)

        # intersection task
        self.try_run_active_obstacle_detector()

        rospy.loginfo("Prepare drive")

    def enable_passive_obstacle_detector(self):
        empty_msg = std_srvs.srv.EmptyRequest()
        self._avoiding_obst_set_passive.call(empty_msg)

    def redirect_state(self) -> str:
        # if self._drive_mode is DriveMode.NONE:
        # raise ValueError('Drive mode is not selected yet!')

        return self.current_outcomes[0]


class FreeDriveStateCallbackPack(ActionCallbackPack.ActionCallbackPack):

    def doneCallback(self, state, result):
        self._state.enable_passive_obstacle_detector()

    def activeCallback(self):
        rospy.loginfo("Drive action server active")


class FreeDriveStateBuilder(SelfieStateBuilder):

    def __init__(self) -> None:
        SelfieStateBuilder.__init__(self, 'task/free_drive',
                                    ["FreeRunOutcome"],
                                    custom_msgs.msg.drivingAction)
        self._callback_pack = FreeDriveStateCallbackPack()
        self.reset()

    def _reset(self) -> None:
        self._state_to_build = FreeDriveState(self.outcomes)

        # change it later

        goal_to_set = custom_msgs.msg.drivingGoal()
        goal_to_set.mode = True

        self._state_to_build.current_goal = goal_to_set
