#!/usr/bin/env python3

from abc import ABC, abstractmethod
from enum import IntEnum
from typing import Callable, Optional
from typing import List

import actionlib
import rospy
import smach
from custom_msgs.enums import ActionEnum
from custom_msgs.enums import RCEnum
from smach.state import State
from smach.user_data import UserData
from std_msgs.msg import Int8
from utils import ActionCallbackPack


class CompetitionID(IntEnum):
    NONE = -1
    PARKING = 0
    OBSTACLE_EVASION = 1


class ControlMode(IntEnum):
    MANUAL = 0
    AUTO = 1


class ModeController:
    def __init__(self, current_state_notifier: Callable[[ControlMode], None]):

        def service_incoming_mode(new_mode: int):
            self._change_mode(new_mode)

        self._control_mode_subscriber = rospy.Subscriber('/state/rc', Int8, service_incoming_mode)

        self._current_state_notifier = current_state_notifier

        self._current_control_mode = ControlMode.AUTO
        self._previous_control_mode = ControlMode.AUTO

    def switch_to_new_mode(self, incoming_control_mode: ControlMode):
        self._previous_control_mode = self._current_control_mode
        self._current_control_mode = incoming_control_mode

        if self._mode_has_changed():
            self.notify_about_control_mode_change()

    def notify_about_control_mode_change(self):
        self._current_state_notifier(self._current_control_mode)

    def _change_mode(self, incoming_mode: int):
        if incoming_mode == RCEnum.RC_MANUAL:
            self.switch_to_new_mode(ControlMode.MANUAL)
        elif incoming_mode == RCEnum.RC_AUTONOMOUS or incoming_mode == RCEnum.RC_HALF_AUTONOMOUS:
            self.switch_to_new_mode(ControlMode.AUTO)

    def _mode_has_changed(self) -> bool:
        return self._current_control_mode != self._previous_control_mode


class SelfieState(smach.State, ABC):

    def __init__(self, state_outcomes: List[str], waiting_timeout: Optional[int] = None):
        smach.State.__init__(self,
                             outcomes=state_outcomes + ['GoToFreeRunState'],
                             input_keys=['state_input_args', 'competition'],
                             output_keys=['state_output_args', 'competition', 'current_state'])
        self._current_outcomes: List[str] = []
        self._is_aborted = False
        self._current_goal: any = None
        self._action_client: Optional[actionlib.SimpleActionClient] = None
        self._callback_pack: ActionCallbackPack = None
        self._waiting_timeout = rospy.Duration()

    @property
    def current_outcomes(self):
        return self._current_outcomes

    @property
    def current_goal(self) -> any:
        return self._current_goal

    @current_goal.setter
    def current_goal(self, new_goal):
        self._current_goal = new_goal

    def wait_for_result(self) -> None:
        self._action_client.wait_for_result(rospy.Duration(200))

    def finish_state(self, done_callback: Callable[[State, UserData], None],
                     state: State, result: UserData) -> None:
        done_callback(state, result)

    def wait_for_server(self) -> None:
        self._action_client.wait_for_server()

    def prepare_action(self, user_data: UserData) -> None:
        rospy.loginfo("Preparing action!")

    def service_state_result(self, userdata: UserData):
        rospy.loginfo("Finishing action!")

    def send_goal(self):
        self._action_client.send_goal(
            goal=self._current_goal,
            done_cb=lambda state, result: self.finish_state(
                self._callback_pack.doneCallback, state, result),
            active_cb=self._callback_pack.activeCallback,
            feedback_cb=self._callback_pack.feedbackCallback,
        )

    @abstractmethod
    def redirect_state(self) -> str: ...

    @abstractmethod
    def setup_goal(self, user_data: UserData) -> None: ...

    def abort_state(self):
        self._is_aborted = True
        self._action_client.cancel_goal()

    def _go_to_default_mode(self) -> str:
        return self.current_outcomes[0]

    def _reset_state_aborted_flag(self) -> None:
        self._is_aborted = False

    def _update_current_state(self, user_data: UserData) -> None:
        user_data.current_state = self

    def _main_algorithm(self, user_data: UserData) -> None:
        self.wait_for_server()
        self.setup_goal(user_data)
        self.prepare_action(user_data)
        self.send_goal()
        self._update_current_state(user_data)
        self.wait_for_result()
        self.service_state_result(user_data)

    def _return_outcome(self) -> str:
        if self._is_aborted:
            self._reset_state_aborted_flag()
            return self._go_to_default_mode()

        return self.redirect_state()

    def execute(self, user_data: UserData) -> str:
        rospy.loginfo("Executing state")
        self._main_algorithm(user_data)

        return self._return_outcome()


class SelfieStateBuilder(ABC):

    def __init__(self, new_action_name: str, new_action_type: type, new_outcomes: List[str] = []):
        self._state_to_build: SelfieState = None
        self._callback_pack: ActionCallbackPack = None
        self.action_name: str = new_action_name
        self.action_type: type = new_action_type
        self.outcomes: List[str] = new_outcomes

    def product_pre_operations(self) -> None:
        pass

    @property
    def outcomes(self) -> List[str]:
        return self._outcomes

    @outcomes.setter
    def outcomes(self, new_outcomes: List[str]) -> None:
        self._outcomes = ['GoToFreeRunState']
        self._outcomes += new_outcomes

    @property
    def callback_pack(self) -> ActionCallbackPack:
        return self._callback_pack

    @callback_pack.setter
    def callback_pack(self, new_callback_pack: ActionCallbackPack) -> None:
        self._callback_pack = new_callback_pack

    def set_callback_pack_to_building_state(self) -> None:
        self._state_to_build._callback_pack = self._callback_pack
        self._state_to_build._callback_pack._state = self._state_to_build

    @property
    def action_name(self) -> str:
        return self._action_name

    @action_name.setter
    def action_name(self, new_action_name: str) -> None:
        self._action_name = new_action_name

    @property
    def action_type(self) -> type:
        return self._action_type

    @action_type.setter
    def action_type(self, new_action_type) -> None:
        self._action_type = new_action_type

    @abstractmethod
    def _reset(self) -> None:
        pass

    def reset(self) -> None:
        self._reset()
        self._reset_action_client()
        self._reset_outcomes()
        self.set_callback_pack_to_building_state()

    def _reset_action_client(self) -> None:
        self._state_to_build._action_client = actionlib.SimpleActionClient(
            self._action_name, self._action_type)

    def _reset_outcomes(self) -> None:
        self._state_to_build._current_outcomes = self.outcomes

    def product(self):
        self.product_pre_operations()
        state = self._state_to_build
        self.reset()
        return state
