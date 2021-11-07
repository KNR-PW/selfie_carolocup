from enum import Enum

class GoalState(Enum):

    NOT_SEND = 1

    SENT = 2

    ABORTED = 3

    SUCCESS = 4
