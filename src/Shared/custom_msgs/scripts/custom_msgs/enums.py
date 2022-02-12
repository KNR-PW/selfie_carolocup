from enum import IntEnum


class LaneControlEnum(IntEnum):
    UNINITIALIZED = 0
    PASSIVE_RIGHT = 1
    ON_RIGHT = 2
    OVERTAKE = 3
    ON_LEFT = 4
    RETURN_RIGHT = 5


class RCEnum(IntEnum):
    RC_UNINITIALIZED = 0
    RC_MANUAL = 1
    RC_HALF_AUTONOMOUS = 2
    RC_AUTONOMOUS = 3


class TypeOfControl(IntEnum):
    NOT_SELECTED = 0
    HALF_AUTONOMOUS = 1
    AUTONOMOUS = 2
    MANUAL = 3


class NavigationEnum(IntEnum):
    UNINITIALIZED = 0
    FOLLOW_RIGHT_UNDISTRACTED = 1
    OBSTACLE_ON_RIGHT = 2
    OBSTACLE_ON_LEFT = 3
    OBSTACLE_ON_BOTH = 4


class GoalState(IntEnum):
    NOT_SEND = 1
    SENT = 2
    ABORTED = 3
    SUCCESS = 4


class ActionEnum(IntEnum):
    ACTION_NONE = 0
    STARTING_PROCEDURE = 1
    FREE_DRIVE = 2
    PARKING_SPOT_SEARCH = 3
    PARK = 4
    INTERSECTION_STOP = 5


class Task(IntEnum):
    TASK_SHIFTING = 0  # Scheduler setting new action
    STARTING_PROCEDURE = 1,
    WAITING_FOR_BUTTON = 2  # Waiting for button press
    GATE_CLOSED = 3  # Starting gate is closed
    STARTING_DRIVE = 4  # Gate opened, car starts dirve
    STARTING_DISTANCE_REACHED = 5  # Car drove given starting distance
    FREE_DRIVE = 6,
    AUTONOMOUS_DRIVE = 7  # Autonomous ride
    STARTING_LINE_DETECTED = 8  # Start line detected - parking zone incomming
    INTERSECTION_STOP_DETECTED = 9  # Priority intersection detected
    EVENT_SENT = 10  # Event is close - need special interaction
    PARKING_SPOT_DETECTION = 11,
    SEARCHING_PARKING_SPOT = 12  # Searching parking spot
    PLACE_INITIALLY_FOUND = 13  # Potential parking spot detected far away
    PLACE_PROPER_FOUND = 14  # Proper parking spot detected and sent
    PLACE_NOT_FOUND = 15  # Parking spot not found. Parking area finished
    PARK = 16,
    APPROACHING_TO_PARKING_SPOT = 17  # Car is approaching to parking spot
    ENTRY_PARKING_SPOT = 18  # Car drives into parking place
    PARKING_IDLE = 19  # Car parked
    EXIT_PARKING_SPOT = 20  # Car drives out of parking place
    PARKING_COMPLETED = 21  # Car ready to further ride
    INTERSECTION = 22,
    APPROACHING_TO_INTERSECTION = 23  # Approaching to intersection
    BLIND_APPROACHING = 24  # Approaching to intersection when we can't see line
    STOP_TIME_ON_INTERSECTION = 25  # Car stopped because of time delay
    STOP_OBSTACLE_ON_INTERSECTION = 26  # Car stopped because of another car on intersection
    PASSING_INTERSECTION = 27  # Intersection clear - car continues ride
