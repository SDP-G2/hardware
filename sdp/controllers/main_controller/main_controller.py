"""main_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 64

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
motors = []
motorsNames = ['FrontLeftMotor', 'FrontRightMotor',  'BackRightMotor','BackLeftMotor']
for i in range(4):
    motors.append(robot.getDevice(motorsNames[i]))
    motors[i].setPosition(float('inf'))
    motors[i].setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    leftSpeed = 2
    rightSpeed = 2
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    motors[0].setVelocity(leftSpeed)
    motors[1].setVelocity(rightSpeed)
    motors[2].setVelocity(leftSpeed)
    motors[3].setVelocity(rightSpeed)
    pass

# Enter here exit cleanup code.
