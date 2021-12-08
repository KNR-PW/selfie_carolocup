import rospy
import rosparam
from rospkg import RosPack

from enum import Enum
import yaml

from std_msgs.msg import String, Float64, Int8

from custom_msgs.msg import Motion, DriveCommand

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


class Park:
    def __init__(self) -> None:
        self.rosPack = RosPack()
        #self.distance_when_spot_found = distance_when_spot_found
        self.distance_when_spot_found = None
        self.params = self.load_params()["park"]
        self.task_state = Task.PARK
        self.parking_stage = 1
        self.run()
        self.place_found = False
        self.distance_data = 0
        #subscribers
        self.dictance_sub = rospy.Subscriber('/selfie_out/motion', Motion, self.distance_callback)
        self.back_sensor_sub = rospy.Subscriber('/back_distance', Float64, self.back_sensor_callback)
        #publishers
        self.drive_park_pub = rospy.Publisher('/drive_park', DriveCommand, queue_size=10)
        self.state_publisher = rospy.Publisher("/state/task", Int8, queue_size=10)
        while self.distance_when_spot_found == None:
            self.get_find_spot_distance()

    def load_params(self):
        with open(self.rosPack.get_path("park") + "/python/config/default.yaml", "r") as config_stream:
            config = yaml.safe_load(config_stream)
        return config

    def get_find_spot_distance(self):
        try:
            if self.current_back_sensor_data != float("inf"):
                self.distance_when_spot_found = self.current_distance
        except Exception as e:
            print("no back sensor data")

    def publish_speed(self, speed, angle=0):
        drive_command =  DriveCommand()
        drive_command.speed = speed
        drive_command.steering_angle_front = angle
        drive_command.steering_angle_rear = angle
        self.drive_park_pub.publish(drive_command)


    def distance_callback(self, motion):
        self.current_distance = motion.distance
        self.current_speed = motion.speed_linear
    
    def back_sensor_callback(self, data):
        self.current_back_sensor_data = data.data

    def run(self):
        self.task_state = Task.APPROACHING_TO_PARKING_SPOT

    def go_to_parking_spot(self):
        if abs(self.current_distance - self.distance_when_spot_found) < self.params["distance_after_place_spot"]:
            self.publish_speed(self.params["approaching_parking_speed"])
        else:
            self.publish_speed(0)
            self.task_state = Task.ENTRY_PARKING_SPOT

    def entry_parking_spot(self):

        if self.parking_stage == 1:
            #zadajemy sterowanie
            angle = self.params["wheel_angle"]
            linear_speed = -1 * self.params["parking_speed"]
            self.publish_speed(linear_speed, angle)
            if self.current_back_sensor_data == float("inf"):
                self.entry_parking_spot = 2

        if self.parking_stage == 2:
            #zadajemy sterowanie
            pass



    
    def park(self):
        pass
    def return_from_parking(self):
        pass

    def loop(self):
        if self.task_state == Task.APPROACHING_TO_PARKING_SPOT:
            self.go_to_parking_spot()
        if self.task_state == Task.ENTRY_PARKING_SPOT:
            self.entry_parking_spot()






if __name__ =="__main__":
    try:
        rospy.init_node('search_parking_spot', anonymous=True)
        rate = rospy.Rate(10)
        s1 = Park()
        s1.load_params()
        while not rospy.is_shutdown():
            s1.loop()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
