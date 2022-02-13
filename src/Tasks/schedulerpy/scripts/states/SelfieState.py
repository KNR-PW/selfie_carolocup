#!/usr/bin/env python3

import smach
import rospy
import actionlib
from abc import ABC, abstractmethod
from typing import List
from smach.user_data import UserData
from smach.state import State
from typing import Callable
from utils import ActionCallbackPack


class SelfieState(smach.State, ABC):

    def __init__(self,
                 state_outcomes: List[str],
                 input_keys: List[str] = [],
                 output_keys: List[str] = []):
        smach.State.__init__(self,
                             outcomes=state_outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self._current_outcomes: List[str] = []
        self._current_goal: any = None
        self._action_client: actionlib.SimpleActionClient = None
        self._callback_pack: ActionCallbackPack = None

    @property
    def current_outcomes(self):
        return self._current_outcomes

    @property
    def current_goal(self) -> any:
        return self._current_goal

    @current_goal.setter
    def current_goal(self, new_goal):
        self._current_goal = new_goal

    def wait_for_result(self):
        self._action_client.wait_for_result()

    def finish_state(self, done_callback: Callable[[State, UserData], None],
                     state: State, result: UserData):
        done_callback(state, result)

    def wait_for_server(self):
        self._action_client.wait_for_server()

    def prepare_action(self, user_data: UserData):
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
    def redirect_state(self):
        pass

    def execute(self, userdata: UserData):
        rospy.loginfo("Executing state")

        self.wait_for_server()
        self.prepare_action(userdata)
        self.send_goal()
        self.wait_for_result()
        self.service_state_result(userdata)

        return self.redirect_state()


class SelfieStateBuilder(ABC):

    def __init__(self, new_action_name: str, new_outcomes: List[str],
                 new_action_type: type):
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
        self._outcomes = new_outcomes

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
