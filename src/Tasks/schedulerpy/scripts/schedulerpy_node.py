import rospy
import smach

from states import StartingState, FreeDriveState, IntersectionState, SelfieState
from states.SelfieState import CompetitionID, ControlMode
from typing import Dict, Optional


class Scheduler:

    def __init__(self):
        self._init_state_machine()
        self._init_mode_controller()

        rospy.loginfo("Scheduler has been created successfully")

    def _init_mode_controller(self):
        def _control_mode_handler_closure(control_mode: ControlMode):
            self._control_mode_handler(control_mode)

        self._mode_controller = SelfieState.ModeController(_control_mode_handler_closure)

    def _init_state_machine(self):
        self._sm = smach.StateMachine(outcomes=["Exit"])
        # global state machine variables available from each state
        self._sm.userdata.in_args = None
        self._sm.userdata.out_args = None
        self._sm.userdata.competition = CompetitionID.NONE
        self._sm.current_state = None

        start_build = StartingState.StartingStateBuilder()
        free_build = FreeDriveState.FreeDriveStateBuilder()
        intersection_builder = IntersectionState.IntersectionStateBuilder()

        self._starting_state: StartingState.StartingState = start_build.product()
        self._free_state: FreeDriveState.FreeDriveState = free_build.product()
        self._intersection_state: IntersectionState.IntersectionState = intersection_builder.product()

    def _control_mode_handler(self, new_control_mode: ControlMode):
        if new_control_mode == ControlMode.MANUAL:
            self._free_state.set_manual_mode()
            self._sm.current_state.abort()
        elif new_control_mode == ControlMode.AUTO:
            self._free_state.set_auto_mode()

    @staticmethod
    def get_standard_remapping() -> Dict[str, str]:
        return {'state_input_args': 'in_args',
                'state_output_args': 'out_args',
                'competition': 'competition',
                'current_state': 'current_state'}

    def __enter__(self):
        rospy.loginfo("Preparing to run...")

        return self

    def __exit__(self, exc_type, exc_value, exc_tb):
        if exc_type is None:
            rospy.loginfo("Scheduler exited normally")
        else:
            print("Raise an exception! " + str(exc_type))
            return False

    def run(self):
        rospy.loginfo("Running scheduler...")

        with self._sm:
            smach.StateMachine.add(
                "StartingState",
                self._starting_state,
                transitions={
                    self._starting_state.current_outcomes[0]: "FreeRunState"
                },
                remapping=Scheduler.get_standard_remapping())

            smach.StateMachine.add(
                "FreeRunState",
                self._free_state,
                transitions={
                    self._free_state.current_outcomes[0]: "FreeRunState",
                    self._free_state.current_outcomes[1]: "IntersectionState",
                    self._free_state.current_outcomes[2]: "IntersectionState"
                },
                remapping=Scheduler.get_standard_remapping())

            smach.StateMachine.add(
                "IntersectionState",
                self._intersection_state,
                transitions={
                    self._intersection_state.current_outcomes[0]: "FreeRunState"
                },
                remapping=Scheduler.get_standard_remapping()
            )

        rospy.loginfo("State machine is running...")
        outcome = self._sm.execute()


if __name__ == "__main__":
    rospy.init_node("schedulerpy")
    rospy.loginfo("Rospy started!\n")

    with Scheduler() as scheduler:
        scheduler.run()
