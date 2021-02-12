from robot import *

# The serial number of the robot
# TODO: Read this from some hardware based source
ROBOT_SERIAL_NUMBER = "123"

def main():
    # Create an instance of the robot
    robot = Robot(ROBOT_SERIAL_NUMBER)

    # Poll the server for the latest commands
    robot.poll()

if __name__ == '__main__':
    main()
