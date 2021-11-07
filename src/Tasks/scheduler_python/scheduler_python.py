#!/usr/bin/env python
"""
 *Copyright ( c ) 2019, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
"""
import rospy
from std_msgs.msg import Int8
import actionlib
from actionlib_msgs.msg import GoalID
from enums.ActionEnum import Action
from enums.GoalStateEnum import GoalState
from enums.TaskEnum import Task
from enums.TypeOfControlEnum import TypeOfControl


from clients.basic_client import BasicClient
from clients.intersection import IntersectionClient
from clients.starting_procedure import StartingProcedureClient
from clients.free_drive import FreeDriveClient
from clients.search_park import SearchClient
from clients.park import ParkClient



class Scheduler():

    def __init__(self) -> None:
        self.active_client = BasicClient("asdf", GoalID)
        rospy.init_node('name')
        #przechowuje klienty
        self.clients = {
            Action.FREE_DRIVE: FreeDriveClient("name", GoalID),
            Action.INTERSECTION_STOP: None,
            Action.PARK: None,
            Action.PARKING_SPOT_SEARCH: None,
            Action.STARTING_PROCEDURE: StartingProcedureClient("name", GoalID)
        }
        #jakis typ nizej
        self.searching_parking_spot_client = BasicClient("parking_spot_search", GoalID)


        #auto czy maunal
        self.current_type_of_control = TypeOfControl.NOT_SELECTED


        #obecny task
        self.current_task_state = Task.WAITING_FOR_BUTTON
        self.next_task_state = self.current_task_state

        #ustawienia startowe jakies
        self.park_counter_= 0
        self.num_park_to_complete_= 2
        self.start_distance_ = 1.0
        self.parking_spot_ = 0.6

        rospy.get_param("starting_distance", self.start_distance_)
        rospy.get_param("parking_spot", self.parking_spot_)
        rospy.get_param("num_park_to_complete", self.num_park_to_complete_)
        
        rospy.loginfo(
        f"Created scheduler with params: PC: {self.num_park_to_complete_} SD: {self.start_distance_}, PS: {self.parking_spot_}")
        
        rospy.Subscriber("/state/task", Int8, self.taskStateCallback)

        rospy.Subscriber("/state/rc", Int8, self.rcStateCallback)

        rospy.loginfo("Clients created successfully")
        
        self.start_new_action(self.clients[Action.STARTING_PROCEDURE])


    def wait_for_start(self):
        while not rospy.is_shutdown():
            if self.get_active_client.goal_state == GoalState.SUCCESS:
                #skonczone przygotowania
                self.active_client.get
                button = None#nie wiem skąd mam wziąć informacje o przycisku
                self.setup_action_clients(button)
                self.active_client.set_action_scenario(button)
            break

    def get_active_client(self):
        return self.active_client

    def taskStateCallback(self, callback):
        self.next_task_state = Task(callback)
            
   
    def rcStateCallback(self, callback):
        self.current_type_of_control = TypeOfControl(callback)
        

    def __del__(self):
        rospy.delete_param("begin_action")
        rospy.delete_param("starting_distance")
        rospy.delete_param("parking_spot")


    def start_new_action(self, client):
        #dodac logike odpowiadającą za włączenie akcji
        pass


    def setup_action_clients(self, button_pressed):
        if (button_pressed == 0):
            self.clients[Action.PARKING_SPOT_SEARCH] = SearchClient() 
            self.clients[Action.PARK] = ParkClient("nazwa")
        else:
            self.clients[Action.INTERSECTION_STOP] = IntersectionClient("nazwa", Int8)
        





    
    def waitForStart(self):
        while not rospy.is_shutdown:
            if self.is_current_action_finished() == GoalState.SUCCESS:
                if self.current_task_state != self.next_task_state:
                    pass
                else:
                    pass
                #włączyc freedrive
            elif self.is_current_action_finished() == GoalState.ABORTED:
                break


    def is_current_action_finished(self):
        return self.active_client.goal_state()


    def stop_action(self):
        rospy.logwarn("STOP current action")
        #dodac stopowanie akcji w cliencie


    def loop(self):
        if self.active_client.goal_state() == GoalState.SUCCESS:
            if self.current_task_state == Action.PARK:
                self.park_counter_ += 1
                if self.park_counter_ >= self.num_park_to_complete_:
                    pass #dodac cos
            if self.next_task_state != self.current_task_state:
                self.start_new_action()
            else:
                self.next_task_state = Action.FREE_DRIVE
                self.start_new_action()

        elif self.active_client.goal_state() == GoalState.ABORTED:
            if self.current_type_of_control == TypeOfControl.MANUAL:
                pass
            else:
                self.stop_action()
                self.next_task_state = Action.FREE_DRIVE
                self.start_new_action()



