from enum import Enum

class TypeOfControl(Enum):
    NOT_SELECTED = 0
    HALF_AUTONOMOUS = 1
    AUTONOMOUS = 2
    MANUAL = 3