from enum import Enum



class Action(Enum):

    ACTION_NONE = 0
    
    STARTING_PROCEDURE = 1

    FREE_DRIVE = 2

    PARKING_SPOT_SEARCH = 3

    PARK = 4

    INTERSECTION_STOP = 5
