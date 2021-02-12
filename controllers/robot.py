import requests

class Robot:
    # Create a new instance of a robot
    # Need no provide the serial number
    def __init__(self, robot_serial_number):
        self.robot_serial_number = robot_serial_number

    # Poll the backend server for more commands
    def poll(self):
        poll_json_body = {"robot_serial_number": str(self.get_robot_serial_number()), "battery_level": int(self.get_battery_level()), "instruction": str(self.get_instruction())}
        poll_response = requests.get("http://localhost:8080/poll", json=poll_json_body)

        # TODO: Do something with the response
        print(poll_response.text)


    # Get the battery level of the robot
    def get_battery_level(self):
        return 90

    # Get the instruction the robot is currently doing
    def get_instruction(self):
        return "Idle"

    # Get the serial number of the robot
    def get_robot_serial_number(self):
        return "serial1"
