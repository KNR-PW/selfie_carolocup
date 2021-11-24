#ros imports
import rospy
import rosparam
from rospkg import RosPack
#non-ros imports
from enum import Enum
import yaml
#file imports
from std_msgs.msg import String
from custom_msgs import Motion



#this enum will be in different file
class Task(Enum):


    TASK_SHIFTING = 0,  #Scheduler setting new action



    STARTING_PROCEDURE = 1,

    WAITING_FOR_BUTTON = 2,         #Waiting for button press

    GATE_CLOSED = 3,                #Starting gate is closed

    STARTING_DRIVE = 4,             #Gate opened, car starts dirve

    STARTING_DISTANCE_REACHED = 5,  #Car drove given starting distance

    FREE_DRIVE = 6,




    AUTONOMOUS_DRIVE = 7,            #Autonomous ride

    STARTING_LINE_DETECTED = 8,      #Start line detected - parking zone incomming

    INTERSECTION_STOP_DETECTED = 9,  #Priority intersection detected

    EVENT_SENT = 10,                  #Event is close - need special interaction

    PARKING_SPOT_DETECTION = 11,

    SEARCHING_PARKING_SPOT = 12,  #Searching parking spot

    PLACE_INITIALLY_FOUND = 13,   #Potential parking spot detected far away

    PLACE_PROPER_FOUND = 14,      #Proper parking spot detected and sent

    PLACE_NOT_FOUND = 15,         #Parking spot not found. Parking area finished

    PARK = 16,




    APPROACHING_TO_PARKING_SPOT = 17,  #Car is approaching to parking spot

    ENTRY_PARKING_SPOT = 18,           #Car drives into parking place

    PARKING_IDLE = 19,                 #Car parked

    EXIT_PARKING_SPOT = 20,            #Car drives out of parking place

    PARKING_COMPLETED = 21,            #Car ready to further ride

    INTERSECTION = 22,




    APPROACHING_TO_INTERSECTION = 23,    #Approaching to intersection

    BLIND_APPROACHING = 24,              #Approaching to intersection when we can't see line

    STOP_TIME_ON_INTERSECTION = 25,      #Car stopped because of time delay

    STOP_OBSTACLE_ON_INTERSECTION = 26,  #Car stopped because of another car on

    
    
    
    #intersection

    PASSING_INTERSECTION = 27            #Intersection clear - car continues ride



class SearchParkingSpot():
    def __init__(self):
        self.rosPack = RosPack()
        # self.params = rosparam.load_file("python/config/default.yaml")
        self.params = self.load_params()["parking_spot_detector"]
        self.is_spot_found = False
        self.distance = 0
        self.task_state = Task.PARKING_SPOT_DETECTION
        
        #Subscibers
        #zmienic typ wiadomosci
        self.dictance_sub = rospy.Subscriber('/selfie_out/motion', Motion, self.distance_callback)
        #subscriber na wiadomosci z 

    def start_searching(self):
        #change speed 
        self.change_task_state(Task.SEARCHING_PARKING_SPOT, "Searching parking spot")


    def load_params(self):
        x = self.rosPack.get_path("parking_spot_detector")
        with open(self.rosPack.get_path("parking_spot_detector") + "/python/config/default.yaml", "r") as config_stream:
            config = yaml.safe_load(config_stream)
        return config

    def change_task_state(self, state, message):
        self.task_state = state
        rospy.loginfo(message)
    
    def is_parking_spot_found(self) -> bool:
        
        #if warunek z czujników -> return True

        return False

    def loop(self):
        if self.params["length_of_parking_area"] < "odleglosc aktualna":
            self.change_task_state(Task.PLACE_NOT_FOUND, "parking spot not found")

        elif self.task_state == Task.SEARCHING_PARKING_SPOT and self.is_parking_spot_found():
            self.change_task_state(Task.PLACE_INITIALLY_FOUND, "parking spot found")
            TODO: odleglosc
            # tu cos wiecej zrobic
            self.change_task_state(Task.PLACE_PROPER_FOUND, "proper parking spot found")
        elif self.task_state == Task.PLACE_NOT_FOUND:
            self.change_task_state(Task.FREE_DRIVE)
            #przestan szukac, włącz freedrive
        elif self.task_state == Task.PLACE_PROPER_FOUND:
            pass

    def distance_callback(self, motion):
        self.current_distance = motion.distance
        
            





if __name__ =="__main__":
    s1 = SearchParkingSpot()
    s1.load_params()
