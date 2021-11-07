import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from enums.GoalStateEnum import GoalState
from enums.ActionEnum import Action

class BasicClient:

    next_action = Action.FREE_DRIVE

    def __init__(self, service_name, message_type) -> None:
        self.goal_state = GoalState.NOT_SEND
        self.task = None
        # rospy.wait_for_service(service_name)
        self.client = actionlib.SimpleActionClient(service_name, message_type)

    def start_action(self, goal):
        self.client.wait_for_server()
        self.goal_state = GoalState.SENT
        self.client.send_goal(goal)
    
    @property
    def goal_state(self):
        return self.__goal_state

    @goal_state.setter
    def goal_state(self, goal_state1 = None):
        if goal_state1 is None:
            client_state = self.client.get_state()
        
        #Mapping GoalState from actionlib to custom Enum
            if client_state == GoalStatus.SUCCEEDED:
                self.goal_state =  GoalState.SUCCESS
            elif client_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST, GoalStatus.RECALLED, GoalStatus.RECALLING]:
                self.goal_state =  GoalState.ABORTED
            elif client_state in [GoalStatus.ACTIVE, GoalStatus.PREEMPTED]:
                self.goal_state =  GoalState.SENT 
            else:
                self.goal_state =  GoalState.NOT_SEND
        else:
            self.__goal_state = goal_state1

    def cancel_action(self):
        rospy.loginfo("canceling goals")
        self.client.cancel_all_goals()

    def wait_for_result(self, duration):
        return self.client.wait_for_result(rospy.Duration(duration))
    
    def wait_for_server(self, duration):
        self.goal_state = GoalState.NOT_SEND
        rospy.loginfo("Wait for driving action server")
        self.client.wait_for_server(rospy.Duration(duration))



