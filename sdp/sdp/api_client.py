import requests
from .command_statuses import CommandStatus


# pub struct Poll {
#     pub robot_serial_number: String,
#     pub current_command_id: i64,
#     pub current_command_status: Status,
#     pub instruction: Instruction,
#     pub battery_level: i64,
# }


class APIClient():

    def __init__(self, serial_number: str):
        self.url = "http://localhost:8080"
        self.robot_serial_number = serial_number

    def poll(self, battery_level: int, command_id: str, command_status: CommandStatus):
        poll_json_body = {
            "robot_serial_number": self.robot_serial_number, 
            "battery_level": battery_level, 
            "current_command_id": command_id,
            "current_command_status": command_status.value 
        }

        poll_response = requests.get("http://localhost:8080/poll", json=poll_json_body)

        return poll_response