from enum import Enum

class TaskTypes(Enum):
    Idle = 'Idle'
    AbortLowBattery ='LowBattery'
    AbortSafety = 'Safety'
    TaskCircular = 'Circular'
    TaskZigZag = 'ZigZag'