import actionlib
import rospy
from clients.basic_client import BasicClient
#nie wiem czy ten import jest okej
# import src.Tasks
from enums.ActionEnum import Action
from enums.GoalStateEnum import GoalState
from std_msgs.msg import Empty 


actionlib.SimpleActionClient

class FreeDriveClient(BasicClient):
    def __init__(self, service_name) -> None:
        super().__init__(service_name, driving)
        #ktory wcisniety przycisk
        self.button = 0
        #nie wiem czy tak mają być te serwisy, czy np. jedna definicja dal wszystkich klientów
        # (bo w pythonie nie znalazlem czegos takiego jak nodehandler)
        self.visionReset_ = rospy.ServiceProxy("resetVision")
        self.resetLaneController_ = rospy.ServiceProxy("resetLaneControl")
        #nie wiem co robic z muxem, bo nie znalazlem topic_tools na pythona i nie wiem jak sie za to zabrac
        self.muxDriveSelect_ = rospy.ServiceProxy("drive_multiplexer/select")
        self.avoidingObstSetPassive_ = rospy.ServiceProxy("avoiding_obst_set_passive")
        self.avoidingObstSetActive_ = rospy.ServiceProxy("avoiding_obst_set_active")
    
    def set_action_scenario(self, mode):
        self.button = mode
        if mode:
            BasicClient.next_action = Action.INTERSECTION_STOP
        else:
            BasicClient.next_action = Action.PARKING_SPOT_SEARCH
        
    def remove_next_action(self):
        BasicClient.next_action = Action.FREE_DRIVE

    def send_goal(self, goal):
        goal.mode = self.button
        self.start_action(goal)
    
    def is_action_done(self):
        self.update_goal_state()
        if self.goal_state == GoalState.ABORTED:
            empty_msg = Empty
            self.avoidingObstSetPassive(empty_msg)
 
    def set_action_clients(self):
        empty_msg = Empty

        self.visionReset_(empty_msg)
        self.resetLaneController_(empty_msg)

        #nie wiem o co chodzi z muxem, ale tutaj chyba tez cos z nim trzeba by zrobic
        if self.button:
            self.avoidingObstSetActive_(empty_msg)
        else:
            self.avoidingObstSetPassive_(empty_msg)
        rospy.loginfo(f"Prepare drive - vision reset, reset Lane control, MuxSelect drive/lane_control,",
        "avoinding obst set {self.button}")