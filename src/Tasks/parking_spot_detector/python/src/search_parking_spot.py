#ros imports
import rospy
import rosparam
from rospkg import RosPack
#non-ros imports
from enum import Enum
import yaml
#msg imports
from std_msgs.msg import String, Float64, Int8
from custom_msgs.msg import Motion


#this enum will be in a different file
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
        self.params = self.load_params()["parking_spot_detector"]
        
        self.task_state = Task.PARKING_SPOT_DETECTION

        #data from /selfie_out/motion
        self.current_distance = None

        #data from /back_distance
        self.back_sensor_data = float("inf")

        self.max_distance_of_parking_area = None

        #data for measuring distance between obstacles
        self.last_data_read_distance = None

        #Subscibers
        self.dictance_sub = rospy.Subscriber('/selfie_out/motion', Motion, self.distance_callback)
        self.back_sensor_sub = rospy.Subscriber('/back_distance', Float64, self.back_sensor_callback )

        #Publishers
        self.state_publisher = rospy.Publisher("/state/task", Int8, queue_size=10)
        self.speed_publisher = rospy.Publisher("/max_speed", Float64, queue_size=10)

        #set speed for parking search
        self.change_speed(self.task_state)


    def back_sensor_callback(self, data):
        self.back_sensor_data = data
        


    def start_searching(self):
        self.change_task_state(Task.SEARCHING_PARKING_SPOT, "Searching parking spot")


    def load_params(self):
        x = self.rosPack.get_path("parking_spot_detector")
        with open(self.rosPack.get_path("parking_spot_detector") + "/python/config/default.yaml", "r") as config_stream:
            config = yaml.safe_load(config_stream)
        return config

    def change_task_state(self, state, message):
        self.task_state = state
        self.state_publisher.publish(state.value)
    
    def is_parking_spot_found(self) -> bool:
        
        if self.back_sensor_data.data != float("inf"):
            if self.current_distance != None:
                self.penultimate_data_read_distance = self.last_data_read_distance if self.last_data_read_distance != None else self.current_distance
                self.last_data_read_distance = self.current_distance
                if self.last_data_read_distance - self.penultimate_data_read_distance > self.params["min_space_between_rectangles"]:
                    return True
                    
        return False

    def change_speed(self, task_state):
        if task_state == Task.PLACE_INITIALLY_FOUND or task_state == Task.PLACE_PROPER_FOUND:
            self.speed_publisher.publish(self.params["speed_when_found_place"])
        elif task_state == Task.SEARCHING_PARKING_SPOT:
            self.speed_publisher.publish(self.params["default_speed_in_parking_zone"])


    def loop(self):
        if self.max_distance_of_parking_area < self.current_distance:
            self.change_task_state(Task.PLACE_NOT_FOUND, "parking spot not found")

        
        elif self.task_state == Task.SEARCHING_PARKING_SPOT and self.is_parking_spot_found():
            self.change_task_state(Task.PLACE_INITIALLY_FOUND, "parking spot found")
            self.change_speed(self.task_state)

            if self.penultimate_data_read_distance < self.max_distance_of_parking_area:
                self.change_task_state(Task.PLACE_PROPER_FOUND, "proper parking spot found")
            else:
                self.change_task_state(Task.PLACE_NOT_FOUND)
        elif self.task_state == Task.PLACE_NOT_FOUND:
            self.change_task_state(Task.FREE_DRIVE)
            #przestan szukac, włącz freedrive
        elif self.task_state == Task.PLACE_PROPER_FOUND:
            #jechanie do miejsca
            pass

    def distance_callback(self, motion):
        self.current_distance = motion.distance
        if self.max_distance_of_parking_area == None and self.task_state == Task.PARKING_SPOT_DETECTION:
            self.max_distance_of_parking_area = self.current_distance + self.params["lengtg_of_parking_area"]
        
            





if __name__ =="__main__":
    try:
        rospy.init_node('search_parking_spot', anonymous=True)
        rate = rospy.Rate(10)
        s1 = SearchParkingSpot()
        while not rospy.is_shutdown():
            s1.loop()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass