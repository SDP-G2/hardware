import requests
import asyncio
import aiohttp
from .command_statuses import CommandStatus
from .task_types import TaskTypes
import requests

# pub struct Poll {
#     pub robot_serial_number: String,
#     pub current_command_id: i64,
#     pub current_command_status: Status,
#     pub instruction: Instruction,
#     pub battery_level: i64,
# }


class APIClient():

    def __init__(self, serial_number: str):
        self.base_url = "http://localhost:8000"
        self.robot_serial_number = serial_number

    async def poll(self, battery_level: int, command_id: str, command_status: CommandStatus = CommandStatus.InProgress):
        poll_json_body = {
            "robot_serial_number": self.robot_serial_number, 
            "battery_level": battery_level, 
            "command_id": command_id,
            "status": command_status.value 
        }

        print(poll_json_body)

        resp = requests.get(self.base_url + "/poll", json=poll_json_body)
        return resp.text

        # async with aiohttp.ClientSession() as session:
        #     async with session.get(self.base_url + "/poll", json=poll_json_body) as resp:
        #         return resp.json()

    def instruction_to_robot_task(self, response):
        if response['instruction'] == TaskTypes.Idle.value:
            return TaskTypes.Idle
            
        task_specifier = list(response['instruction'].values())[0]
        return TaskTypes(task_specifier)

    async def init(self, battery_level: int):
        init_json_body = {
            "robot_serial_number": self.robot_serial_number, 
            "battery_level": battery_level
        }

        resp = requests.get(self.base_url + "/init", json=init_json_body)
        return resp.text
        # async with aiohttp.ClientSession() as session:
        #     async with session.get(self.base_url + "/init", json=init_json_body) as resp:
        #         return resp.text