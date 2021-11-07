from enum import Enum


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

