import imp
import smach
from std_msgs.msg import Int8
import smach_ros
import rospy

# from core  import scheduler

import threading
import asyncio

import actionlib
import topic_tools.srv

# import custom_msgs.msg

import std_srvs.srv

from states import StartingState, FreeDriveState, IntersectionState


class Scheduler:

    def rcCb(self, msg):
        rospy.get_param()

    def __init__(self):
        rospy.loginfo("Scheduler has been created successfuly")

        # self.state_notifier = rospy.Subscriber("/state/task", Int8, cb)

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

        sm = smach.StateMachine(outcomes=["Exit"])
        sm.userdata.goal_data = 0
        sm.userdata.previous_state = 'None'

        start_build = StartingState.StartingStateBuilder()
        free_build = FreeDriveState.FreeDriveStateBuilder()

        startingState = start_build.product()
        freeState = free_build.product()
        intersectionState = IntersectionState.IntersectionStateBuilder(
        ).product()

        with sm:
            smach.StateMachine.add("StartingState",
                                   startingState,
                                   transitions={
                                       startingState.current_outcomes[0]:
                                       "FreeRunState"
                                   },
                                   remapping={
                                       'state_output': 'goal_data',
                                       'prev_state': 'previous_state'
                                   })
            smach.StateMachine.add(
                "FreeRunState",
                freeState,
                transitions={
                    freeState.current_outcomes[0]: "IntersectionState"
                },
                remapping={
                    'state_input': 'goal_data'  #,
                    #    'prev_state': 'previous_state'
                })
            smach.StateMachine.add(
                "IntersectionState",
                intersectionState,
                transitions={
                    intersectionState.current_outcomes[0]: "FreeRunState"
                },
                #    remapping={'prev_state': 'previous_state'}
            )
        rospy.loginfo("State machine is running...")
        outcome = sm.execute()


if __name__ == "__main__":
    rospy.init_node("schedulerpy")
    rospy.loginfo("Rospy started!\n")

    with Scheduler() as scheduler:
        scheduler.run()
