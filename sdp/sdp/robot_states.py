from enum import Enum

class RobotStates(Enum):
    Init = 0
    Idle = 1
    Task = 2
    PostTask = 3