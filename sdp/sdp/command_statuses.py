from enum import Enum

class CommandStatus(Enum):
    Pending    = 'Pending'    # Preparation for task
    InProgress = 'InProgress' # InProgress
    Paused     = 'Paused'     # Blocked by obstacle
    Completed  = 'Completed'  # Finished: Task -> PostTask # Mark as completed
    Cancelled  = 'Cancelled'  # When aborted